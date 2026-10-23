// ESP-IDF firmware for esp32_motor_hardware_interface.
//
// Talks to the Jetson over UART2 (GPIO16 RX / GPIO17 TX) and drives two DC motors through
// both channels of an L298N. Each motor's quadrature encoder is counted in hardware by a PCNT
// unit. UART0 (USB) is left free for flashing and ESP_LOG output.
//
// Protocol (one line in, one line out, 115200 8N1):
//   Jetson -> ESP32:  "V <v1> <v2>\n"              target velocities, rad/s
//   ESP32 -> Jetson:  "S <p1> <v1> <p2> <v2>\n"    measured positions (rad) and velocities (rad/s)
//   Jetson -> ESP32:  "G <kp> <ki> <kd>\n"         PID gains, applied to both motors
//   ESP32 -> Jetson:  "OK\n"
//                     "ERR <message>\n"            on a bad command
//
// Written for ESP-IDF v5.x.

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "esp32_motor";

// ---- UART link to the Jetson ------------------------------------------------------------
#define LINK_UART UART_NUM_2
#define LINK_RX_PIN GPIO_NUM_16  // to Jetson header pin 8 (TX)
#define LINK_TX_PIN GPIO_NUM_17  // to Jetson header pin 10 (RX)
#define LINK_BAUD 115200
#define LINK_MAX_LINE 64

// ---- Control --------------------------------------------------------------------------------
#define PWM_FREQ_HZ 20000
#define PWM_RESOLUTION LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY ((1 << 10) - 1)
#define CONTROL_PERIOD_MS 10
#define WATCHDOG_US (500 * 1000)  // stop if the Jetson goes quiet
#define VELOCITY_FILTER 0.3f      // 0..1, higher = less smoothing

// Default PID gains until the Jetson sends "G". The ROS side sends its own on startup.
#define DEFAULT_KP 0.04f  // duty per rad/s of error
#define DEFAULT_KI 0.10f  // duty per rad of accumulated error
#define DEFAULT_KD 0.0f   // duty per rad/s^2 of measured acceleration

#define PCNT_LIMIT 30000  // hardware counter range; overflows are accumulated by the driver
#define TWO_PI_F 6.28318530718f
#define MOTOR_COUNT 2

typedef struct {
    // Wiring: change these to match your board
    gpio_num_t enable_pin;  // L298N ENA / ENB (PWM)
    gpio_num_t in_a_pin;    // L298N IN1 / IN3
    gpio_num_t in_b_pin;    // L298N IN2 / IN4
    gpio_num_t encoder_a_pin;
    gpio_num_t encoder_b_pin;
    ledc_channel_t pwm_channel;

    // Measure these for each motor:
    //   counts_per_rev = encoder pulses per motor rev * 4 * gear ratio
    float counts_per_rev;
    bool encoder_inverted;  // flip if positive command reads negative velocity
    float max_velocity;     // rad/s at full duty (measure with a 100% test)

    // Runtime state
    pcnt_unit_handle_t encoder;
    int last_count;
    float integral;
    float previous_measured;
    float target_velocity;    // shared, guarded by state_lock
    float measured_velocity;  // shared, guarded by state_lock
    float position;           // shared, guarded by state_lock
} motor_t;

// Avoid GPIO 0, 2, 5, 12, 15 (boot strapping), 6-11 (flash) and 34-39 (input only, no pull-ups).
static motor_t motors[MOTOR_COUNT] = {
    {
        // Motor 1, L298N channel A
        .enable_pin = GPIO_NUM_25,
        .in_a_pin = GPIO_NUM_26,
        .in_b_pin = GPIO_NUM_27,
        .encoder_a_pin = GPIO_NUM_32,
        .encoder_b_pin = GPIO_NUM_33,
        .pwm_channel = LEDC_CHANNEL_0,
        .counts_per_rev = 11.0f * 4.0f * 30.0f,
        .encoder_inverted = false,
        .max_velocity = 20.0f,
        .encoder = NULL,
        .last_count = 0,
        .integral = 0.0f,
        .previous_measured = 0.0f,
        .target_velocity = 0.0f,
        .measured_velocity = 0.0f,
        .position = 0.0f,
    },
    {
        // Motor 2, L298N channel B
        .enable_pin = GPIO_NUM_4,
        .in_a_pin = GPIO_NUM_18,
        .in_b_pin = GPIO_NUM_19,
        .encoder_a_pin = GPIO_NUM_22,
        .encoder_b_pin = GPIO_NUM_23,
        .pwm_channel = LEDC_CHANNEL_1,
        .counts_per_rev = 11.0f * 4.0f * 30.0f,
        .encoder_inverted = false,
        .max_velocity = 20.0f,
        .encoder = NULL,
        .last_count = 0,
        .integral = 0.0f,
        .previous_measured = 0.0f,
        .target_velocity = 0.0f,
        .measured_velocity = 0.0f,
        .position = 0.0f,
    },
};

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

static portMUX_TYPE state_lock = portMUX_INITIALIZER_UNLOCKED;
static int64_t last_command_us = 0;  // guarded by state_lock
static pid_gains_t gains = {
    .kp = DEFAULT_KP,
    .ki = DEFAULT_KI,
    .kd = DEFAULT_KD,
};  // guarded by state_lock

// ---- Motor / encoder ------------------------------------------------------------------------

static void SetMotor(const motor_t* m, float duty) {
    duty = fmaxf(-1.0f, fminf(1.0f, duty));
    // Both inputs low = coast
    gpio_set_level(m->in_a_pin, duty > 0.0f);
    gpio_set_level(m->in_b_pin, duty < 0.0f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, m->pwm_channel, (uint32_t)(fabsf(duty) * PWM_MAX_DUTY));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, m->pwm_channel);
}

static int ReadEncoder(const motor_t* m) {
    int count = 0;
    pcnt_unit_get_count(m->encoder, &count);
    return m->encoder_inverted ? -count : count;
}

static bool OnEncoderLimit(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t* event, void* ctx) {
    // Nothing to do: registering a callback makes the driver install its ISR, which is what
    // accumulates the count across hardware overflows (accum_count).
    return false;
}

static void InitEncoder(motor_t* m) {
    pcnt_unit_config_t unit_config = {
        .low_limit = -PCNT_LIMIT,
        .high_limit = PCNT_LIMIT,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &m->encoder));

    pcnt_glitch_filter_config_t filter_config = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(m->encoder, &filter_config));

    // Full 4x quadrature decoding: each channel counts both edges of one signal,
    // with the other signal deciding the direction.
    pcnt_chan_config_t chan_a_config = {.edge_gpio_num = m->encoder_a_pin, .level_gpio_num = m->encoder_b_pin};
    pcnt_chan_config_t chan_b_config = {.edge_gpio_num = m->encoder_b_pin, .level_gpio_num = m->encoder_a_pin};
    pcnt_channel_handle_t chan_a = NULL;
    pcnt_channel_handle_t chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(m->encoder, &chan_a_config, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(m->encoder, &chan_b_config, &chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m->encoder, PCNT_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(m->encoder, -PCNT_LIMIT));
    pcnt_event_callbacks_t callbacks = {.on_reach = OnEncoderLimit};
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(m->encoder, &callbacks, NULL));

    // Open-collector encoders need pull-ups; push-pull ones don't mind them.
    gpio_pullup_en(m->encoder_a_pin);
    gpio_pullup_en(m->encoder_b_pin);

    ESP_ERROR_CHECK(pcnt_unit_enable(m->encoder));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(m->encoder));
    ESP_ERROR_CHECK(pcnt_unit_start(m->encoder));
}

static void InitMotor(motor_t* m) {
    gpio_config_t direction_config = {
        .pin_bit_mask = (1ULL << m->in_a_pin) | (1ULL << m->in_b_pin),
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&direction_config));

    ledc_channel_config_t pwm_config = {
        .gpio_num = m->enable_pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = m->pwm_channel,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_config));

    InitEncoder(m);
    SetMotor(m, 0.0f);
}

// ---- Control loop ---------------------------------------------------------------------------

static void UpdateMotor(motor_t* m, float dt, bool watchdog_expired) {
    const int count = ReadEncoder(m);
    const float raw_velocity = (count - m->last_count) / m->counts_per_rev * TWO_PI_F / dt;
    m->last_count = count;

    taskENTER_CRITICAL(&state_lock);
    if (watchdog_expired) {
        m->target_velocity = 0.0f;
    }
    m->measured_velocity += VELOCITY_FILTER * (raw_velocity - m->measured_velocity);
    m->position = count / m->counts_per_rev * TWO_PI_F;
    const float target = m->target_velocity;
    const float measured = m->measured_velocity;
    const pid_gains_t pid = gains;
    taskEXIT_CRITICAL(&state_lock);

    // D acts on the measurement rather than the error, so a new target doesn't cause a kick.
    const float derivative = -(measured - m->previous_measured) / dt;
    m->previous_measured = measured;

    if (target == 0.0f) {
        m->integral = 0.0f;
        SetMotor(m, 0.0f);
        return;
    }

    // Feed-forward gets close, PID removes the remaining error.
    const float error = target - measured;
    if (pid.ki > 0.0f) {
        // Limit the I term to full duty so it can't wind up past what the motor can do.
        m->integral = fmaxf(-1.0f / pid.ki, fminf(1.0f / pid.ki, m->integral + error * dt));
    } else {
        m->integral = 0.0f;
    }
    SetMotor(m, target / m->max_velocity + pid.kp * error + pid.ki * m->integral + pid.kd * derivative);
}

static void ControlTask(void* arg) {
    TickType_t wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();

    while (true) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        const int64_t now_us = esp_timer_get_time();
        const float dt = (now_us - last_us) / 1e6f;
        last_us = now_us;

        taskENTER_CRITICAL(&state_lock);
        const bool watchdog_expired = now_us - last_command_us > WATCHDOG_US;
        taskEXIT_CRITICAL(&state_lock);

        for (int i = 0; i < MOTOR_COUNT; ++i) {
            UpdateMotor(&motors[i], dt, watchdog_expired);
        }
    }
}

// ---- Protocol -------------------------------------------------------------------------------

static void SendLine(const char* line) {
    uart_write_bytes(LINK_UART, line, strlen(line));
    uart_write_bytes(LINK_UART, "\n", 1);
}

// Parses `count` whitespace-separated floats. Returns false if there are fewer.
static bool ParseFloats(const char* cursor, float* values, int count) {
    for (int i = 0; i < count; ++i) {
        char* end = NULL;
        values[i] = strtof(cursor, &end);
        if (end == cursor) {
            return false;
        }
        cursor = end;
    }
    return true;
}

static void HandleGains(const char* args) {
    float values[3];
    if (!ParseFloats(args, values, 3)) {
        SendLine("ERR expected kp ki kd");
        return;
    }
    for (int i = 0; i < 3; ++i) {
        if (!isfinite(values[i]) || values[i] < 0.0f) {
            SendLine("ERR gains must be >= 0");
            return;
        }
    }

    taskENTER_CRITICAL(&state_lock);
    gains.kp = values[0];
    gains.ki = values[1];
    gains.kd = values[2];
    taskEXIT_CRITICAL(&state_lock);

    ESP_LOGI(TAG, "gains: kp=%.4f ki=%.4f kd=%.4f", values[0], values[1], values[2]);
    SendLine("OK");
}

static void HandleVelocity(const char* args) {
    char reply[128];

    float targets[MOTOR_COUNT];
    if (!ParseFloats(args, targets, MOTOR_COUNT)) {
        snprintf(reply, sizeof(reply), "ERR expected %d velocities", MOTOR_COUNT);
        SendLine(reply);
        return;
    }

    float positions[MOTOR_COUNT];
    float velocities[MOTOR_COUNT];
    taskENTER_CRITICAL(&state_lock);
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        motor_t* m = &motors[i];
        m->target_velocity = fmaxf(-m->max_velocity, fminf(m->max_velocity, targets[i]));
        positions[i] = m->position;
        velocities[i] = m->measured_velocity;
    }
    last_command_us = esp_timer_get_time();
    taskEXIT_CRITICAL(&state_lock);

    int length = snprintf(reply, sizeof(reply), "S");
    for (int i = 0; i < MOTOR_COUNT; ++i) {
        length += snprintf(reply + length, sizeof(reply) - length, " %.5f %.5f", positions[i], velocities[i]);
    }
    SendLine(reply);
}

static void HandleLine(const char* line) {
    if (strncmp(line, "V ", 2) == 0) {
        HandleVelocity(line + 1);
    } else if (strncmp(line, "G ", 2) == 0) {
        HandleGains(line + 1);
    } else {
        SendLine("ERR unknown command");
    }
}

static void LinkTask(void* arg) {
    char line[LINK_MAX_LINE];
    size_t line_length = 0;
    uint8_t buffer[64];

    while (true) {
        // Block for the first byte, then take whatever else is already buffered.
        int count = uart_read_bytes(LINK_UART, buffer, 1, portMAX_DELAY);
        if (count <= 0) {
            continue;
        }
        size_t buffered = 0;
        uart_get_buffered_data_len(LINK_UART, &buffered);
        if (buffered > 0) {
            const size_t room = sizeof(buffer) - 1;
            count += uart_read_bytes(LINK_UART, buffer + 1, buffered < room ? buffered : room, 0);
        }

        for (int i = 0; i < count; ++i) {
            const char c = (char)buffer[i];
            if (c == '\n') {
                if (line_length > 0 && line[line_length - 1] == '\r') {
                    --line_length;
                }
                line[line_length] = '\0';
                if (line_length > 0) {
                    HandleLine(line);
                }
                line_length = 0;
            } else if (line_length < sizeof(line) - 1) {
                line[line_length++] = c;
            }
        }
    }
}

// ---- Entry point ----------------------------------------------------------------------------

void app_main(void) {
    const uart_config_t uart_config = {
        .baud_rate = LINK_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART, LINK_TX_PIN, LINK_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    for (int i = 0; i < MOTOR_COUNT; ++i) {
        InitMotor(&motors[i]);
    }

    xTaskCreate(ControlTask, "control", 4096, NULL, 10, NULL);
    xTaskCreate(LinkTask, "link", 4096, NULL, 9, NULL);

    ESP_LOGI(TAG, "ready: %d motors, UART%d at %d baud", MOTOR_COUNT, LINK_UART, LINK_BAUD);
}
