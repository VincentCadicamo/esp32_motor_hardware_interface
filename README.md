# esp32_motor_hardware_interface

A ros2_control hardware interface (ROS 2 Humble) for two DC motors driven by an ESP32. The
ESP32 drives both channels of an L298N and reads a quadrature encoder on each motor. It talks
to a Jetson over the UART on the 40-pin header.

```
ros2_control (Jetson)                         ESP32
  velocity_controller ──► Esp32System ──UART──► PI speed loops ──► L298N ──► motor 1, motor 2
  joint_state_broadcaster ◄───────────────────── encoders ◄─────────────────┘
```

The joints are `motor_1_joint` (L298N channel A) and `motor_2_joint` (channel B). The ROS side
handles however many joints the URDF lists; the firmware is set up for two (`MOTOR_COUNT`).

## Wiring

| From                     | To                          |
|--------------------------|-----------------------------|
| Jetson pin 8 (UART TX)   | ESP32 GPIO16 (RX2)          |
| Jetson pin 10 (UART RX)  | ESP32 GPIO17 (TX2)          |
| Jetson pin 6 (GND)       | ESP32 GND                   |
| ESP32 GND                | L298N GND (common ground)   |

| Signal        | Motor 1               | Motor 2               |
|---------------|-----------------------|-----------------------|
| L298N enable  | GPIO25 → ENA          | GPIO4 → ENB           |
| L298N inputs  | GPIO26 / 27 → IN1/IN2 | GPIO18 / 19 → IN3/IN4 |
| Encoder A / B | GPIO32 / 33           | GPIO22 / 23           |

Remove both ENA and ENB jumpers on the L298N. Pins are set in the `motors[]` table at the top
of the firmware.

> **3.3 V only.** ESP32 pins are not 5 V tolerant. Power the encoder from 3.3 V, or put a
> level shifter or divider on A/B if it has to run at 5 V. The Jetson header UART is 3.3 V,
> so it connects to the ESP32 directly.

## Protocol

115200 baud, 8N1, one line in and one line out:

| Jetson → ESP32 | ESP32 → Jetson          |
|----------------|-------------------------|
| `V <v1> <v2>`  | `S <p1> <v1> <p2> <v2>` |
| `G <kp> <ki> <kd>` | `OK`                |
| anything else  | `ERR <message>`         |

Velocities are in rad/s and positions in rad, in joint order. The ESP32 stops both motors if it
gets no command for 500 ms.

## Firmware

`firmware/esp32_motor` is an ESP-IDF v5.x project for an ESP32-WROOM-32(UE). The encoders are
counted in hardware by the ESP32's PCNT units. Before flashing, fill in each motor's row in the
`motors[]` table in `main/main.c`:

- counts/rev: encoder pulses per motor rev × 4 × gear ratio
- inverted: flip it if a positive command reads back as negative velocity
- max rad/s: output shaft speed at full duty

Install ESP-IDF once, on whichever machine the ESP32's USB cable plugs into (Mac or Jetson):

```bash
# macOS: brew install cmake ninja dfu-util python3
# Ubuntu/Jetson: sudo apt install git wget flex bison gperf python3 python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
mkdir -p ~/esp && cd ~/esp
git clone -b v5.3.2 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf && ./install.sh esp32
```

Build and flash (run `. ~/esp/esp-idf/export.sh` in each new terminal first):

```bash
cd firmware/esp32_motor
idf.py set-target esp32          # first time only
idf.py build
idf.py -p <port> flash monitor   # Mac: /dev/cu.usbserial-*, Linux: /dev/ttyUSB0
```

`monitor` shows the ESP32's log on USB; it should print `ready: 2 motors`. Press `Ctrl+]` to
exit.

## Jetson setup

```bash
# Let your user open the UART
sudo usermod -aG dialout $USER        # then log out and back in

# On Jetson Nano the header UART runs a serial console; turn it off
sudo systemctl stop nvgetty && sudo systemctl disable nvgetty
```

The header UART is `/dev/ttyTHS1` on Jetson Nano and Xavier. Some Orin images use
`/dev/ttyTHS0`. Pass `device:=...` if yours is different.

## Build and run

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this repo> esp32_motor_hardware_interface
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select esp32_motor_hardware_interface
source install/setup.bash

ros2 launch esp32_motor_hardware_interface motor.launch.py            # device:=/dev/ttyTHS0
```

In another terminal:

```bash
# Motor 1 at 5 rad/s, motor 2 at -3 rad/s (one value per joint, in order)
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [5.0, -3.0]"

# Watch positions / velocities
ros2 topic echo /joint_states

# Stop both
ros2 topic pub --once /velocity_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0]"
```

The controller ignores messages that don't have exactly one value per joint.

## Tuning the PID gains

The ESP32's speed loop gains are live ROS parameters (`kp`, `ki`, `kd`) on the node
`esp32_motor_pid`. Both motors share the same gains. With the launch file running:

```bash
# Sliders for kp / ki / kd
ros2 run rqt_reconfigure rqt_reconfigure        # pick "esp32_motor_pid" on the left

# Commanded vs measured speed, live
ros2 run rqt_plot rqt_plot /joint_states/velocity[0] /joint_states/velocity[1]
```

You can also set a gain from the command line: `ros2 param set /esp32_motor_pid kp 0.06`.

A simple way to tune:

1. Set `ki` and `kd` to 0. Step the target speed (for example between `[2.0, 2.0]` and
   `[6.0, 6.0]`) and raise `kp` until the speed responds quickly without oscillating.
2. Raise `ki` until the speed settles on the target instead of sitting slightly below it.
3. Leave `kd` at 0 unless the speed overshoots. A little `kd` damps overshoot, but too much
   amplifies encoder noise.

Changes are lost on restart. When you're happy, copy the values into the `kp` / `ki` / `kd`
params in `description/urdf/motor.urdf.xacro`.

## Troubleshooting

- **`Could not open /dev/ttyTHS1`**: wrong device, or your user isn't in `dialout`.
- **`ESP32 ... did not answer`**: TX/RX swapped, no common GND, firmware not flashed, or
  `nvgetty` still holds the port.
- **A motor runs away or oscillates**: check that motor's `inverted` flag first, then lower
  `KP` / `KI`.
- **The wrong motor moves**: swap the two rows in the firmware's `motors[]` table.
