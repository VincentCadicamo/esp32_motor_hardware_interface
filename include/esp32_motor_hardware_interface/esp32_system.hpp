///
/// \file 				esp32_system.hpp
/// \author 			Vincent Cadicamo (www.vincentcadicamo.dev) <vcadicamo@icloud.com>
/// \edited             n/a
/// \created			2026-10-22
/// \last-modified		2026-10-22
/// \brief 				ros2_control SystemInterface for motors driven by an ESP32
/// \details
///		Exposes one velocity command interface and position / velocity state interfaces per
///		joint in the URDF. Each write() sends "V <v1> <v2> ..." (rad/s) and the ESP32 answers
///		"S <p1> <v1> <p2> <v2> ..." (rad, rad/s) in the same joint order.
///
///		The ESP32's PID gains are exposed as the parameters kp / ki / kd on the node
///		"esp32_motor_pid", so they can be tuned live with rqt_reconfigure.
///		See README.md in root dir for more info.

#ifndef ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_SYSTEM_HPP
#define ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_SYSTEM_HPP

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "esp32_motor_hardware_interface/esp32_interface.hpp"

namespace vc {
    namespace esp32_motor_hardware_interface {

        /// \brief      ros2_control hardware plugin for ESP32-driven motors
        class Esp32System : public hardware_interface::SystemInterface {

        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(Esp32System)

            ~Esp32System() override;

            hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

            hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

            hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

            hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        private:
            /// \brief      Sends one velocity per joint and stores the state the ESP32 replies with
            /// \returns    true if the ESP32 answered with a valid state line
            bool Exchange(const std::vector<double>& velocities);

            /// \brief      Sends zero velocity to every joint
            bool Stop();

            /// \brief      Sends the current PID gains to the ESP32
            bool SendGains();

            /// \brief      Creates the "esp32_motor_pid" node and spins it on its own thread
            void StartTuningNode();

            /// \brief      Stops the tuning thread and destroys the node
            void StopTuningNode();

            /// \brief      Serial link, created in on_configure
            std::unique_ptr<Esp32Interface> esp32_;

            /// \brief      Hardware parameters from the URDF <ros2_control> tag
            std::string device_;
            uint32_t baud_rate_ = 115200;
            int32_t timeout_ms_ = 20;

            /// \brief      Consecutive failed exchanges before reporting an error
            int max_missed_replies_ = 5;
            int missed_replies_ = 0;

            /// \brief      Interface values shared with the controllers, one entry per joint
            std::vector<double> command_velocities_;
            std::vector<double> state_positions_;
            std::vector<double> state_velocities_;

            /// \brief      Latest state received from the ESP32, copied out in read()
            std::vector<double> received_positions_;
            std::vector<double> received_velocities_;

            /// \brief      PID gains, written by the parameter callback and sent from write()
            std::mutex gains_mutex_;
            double kp_ = 0.04;
            double ki_ = 0.10;
            double kd_ = 0.0;
            std::atomic<bool> gains_dirty_{false};

            /// \brief      Parameter node for live tuning
            rclcpp::Node::SharedPtr tuning_node_;
            rclcpp::executors::SingleThreadedExecutor::SharedPtr tuning_executor_;
            rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
            std::thread tuning_thread_;
            std::atomic<bool> tuning_running_{false};
        };

    } // namespace esp32_motor_hardware_interface
}

#endif //ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_SYSTEM_HPP
