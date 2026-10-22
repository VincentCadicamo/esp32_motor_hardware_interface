///
/// \file 				esp32_system.cpp
/// \author 			Vincent Cadicamo (www.vincentcadicamo.dev) <vcadicamo@icloud.com>
/// \edited             n/a
/// \created			2026-10-22
/// \last-modified		2026-10-22
/// \brief 				ros2_control SystemInterface for motors driven by an ESP32
/// \details
///		See README.md in root dir for more info.

#include "esp32_motor_hardware_interface/esp32_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace vc {
    namespace esp32_motor_hardware_interface {

        namespace {

            const rclcpp::Logger kLogger = rclcpp::get_logger("Esp32System");

            std::string GetParam(const hardware_interface::HardwareInfo& info,
                                 const std::string& name, const std::string& fallback) {
                const auto it = info.hardware_parameters.find(name);
                return it != info.hardware_parameters.end() ? it->second : fallback;
            }

        } // namespace

        hardware_interface::CallbackReturn Esp32System::on_init(const hardware_interface::HardwareInfo& info) {
            if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (info_.joints.empty()) {
                RCLCPP_FATAL(kLogger, "No joints defined in the <ros2_control> tag");
                return hardware_interface::CallbackReturn::ERROR;
            }

            for (const auto& joint : info_.joints) {
                if (joint.command_interfaces.size() != 1 ||
                    joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                    RCLCPP_FATAL(kLogger, "Joint '%s' needs exactly one 'velocity' command interface", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
                if (joint.state_interfaces.size() != 2 ||
                    joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
                    joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                    RCLCPP_FATAL(kLogger, "Joint '%s' needs 'position' then 'velocity' state interfaces", joint.name.c_str());
                    return hardware_interface::CallbackReturn::ERROR;
                }
            }

            try {
                device_ = GetParam(info_, "device", "/dev/ttyTHS1");
                baud_rate_ = static_cast<uint32_t>(std::stoul(GetParam(info_, "baud_rate", "115200")));
                timeout_ms_ = static_cast<int32_t>(std::stoi(GetParam(info_, "timeout_ms", "20")));
                max_missed_replies_ = std::stoi(GetParam(info_, "max_missed_replies", "5"));
            } catch (const std::exception& e) {
                RCLCPP_FATAL(kLogger, "Invalid hardware parameter: %s", e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }

            const size_t joint_count = info_.joints.size();
            command_velocities_.assign(joint_count, 0.0);
            state_positions_.assign(joint_count, 0.0);
            state_velocities_.assign(joint_count, 0.0);
            received_positions_.assign(joint_count, 0.0);
            received_velocities_.assign(joint_count, 0.0);

            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn Esp32System::on_configure(const rclcpp_lifecycle::State&) {
            try {
                esp32_ = std::make_unique<Esp32Interface>(device_, baud_rate_);
                esp32_->SetTimeout(timeout_ms_);
                esp32_->Connect();
            } catch (const std::exception& e) {
                RCLCPP_ERROR(kLogger, "Could not open %s: %s", device_.c_str(), e.what());
                esp32_.reset();
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Stop the motors and confirm the ESP32 is actually answering.
            if (!Stop()) {
                RCLCPP_ERROR(kLogger, "ESP32 on %s did not answer. Check TX/RX wiring, GND and baud rate (%u).",
                             device_.c_str(), baud_rate_);
                esp32_.reset();
                return hardware_interface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(kLogger, "Connected to ESP32 on %s at %u baud (%zu joints)",
                        device_.c_str(), baud_rate_, info_.joints.size());
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn Esp32System::on_cleanup(const rclcpp_lifecycle::State&) {
            esp32_.reset();
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn Esp32System::on_activate(const rclcpp_lifecycle::State&) {
            std::fill(command_velocities_.begin(), command_velocities_.end(), 0.0);
            missed_replies_ = 0;
            if (!Stop()) {
                RCLCPP_ERROR(kLogger, "ESP32 did not answer on activate");
                return hardware_interface::CallbackReturn::ERROR;
            }
            state_positions_ = received_positions_;
            state_velocities_ = received_velocities_;
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        hardware_interface::CallbackReturn Esp32System::on_deactivate(const rclcpp_lifecycle::State&) {
            std::fill(command_velocities_.begin(), command_velocities_.end(), 0.0);
            if (esp32_ && !Stop()) {
                RCLCPP_WARN(kLogger, "Could not send stop on deactivate; ESP32 watchdog will stop the motors");
            }
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> Esp32System::export_state_interfaces() {
            std::vector<hardware_interface::StateInterface> interfaces;
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                const std::string& joint = info_.joints[i].name;
                interfaces.emplace_back(joint, hardware_interface::HW_IF_POSITION, &state_positions_[i]);
                interfaces.emplace_back(joint, hardware_interface::HW_IF_VELOCITY, &state_velocities_[i]);
            }
            return interfaces;
        }

        std::vector<hardware_interface::CommandInterface> Esp32System::export_command_interfaces() {
            std::vector<hardware_interface::CommandInterface> interfaces;
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &command_velocities_[i]);
            }
            return interfaces;
        }

        hardware_interface::return_type Esp32System::read(const rclcpp::Time&, const rclcpp::Duration&) {
            // The serial exchange happens in write(); this publishes the last reply.
            state_positions_ = received_positions_;
            state_velocities_ = received_velocities_;
            return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type Esp32System::write(const rclcpp::Time&, const rclcpp::Duration&) {
            std::vector<double> velocities(command_velocities_.size());
            for (size_t i = 0; i < velocities.size(); ++i) {
                velocities[i] = std::isfinite(command_velocities_[i]) ? command_velocities_[i] : 0.0;
            }

            if (Exchange(velocities)) {
                missed_replies_ = 0;
                return hardware_interface::return_type::OK;
            }

            // Tolerate the odd dropped reply; only fail when the link is really gone.
            if (++missed_replies_ >= max_missed_replies_) {
                RCLCPP_ERROR(kLogger, "ESP32 missed %d replies in a row", missed_replies_);
                return hardware_interface::return_type::ERROR;
            }
            return hardware_interface::return_type::OK;
        }

        bool Esp32System::Stop() {
            return Exchange(std::vector<double>(info_.joints.size(), 0.0));
        }

        bool Esp32System::Exchange(const std::vector<double>& velocities) {
            std::string command = "V";
            char value[24];
            for (const double velocity : velocities) {
                std::snprintf(value, sizeof(value), " %.4f", velocity);
                command += value;
            }

            Esp32Response response;
            try {
                response = esp32_->SendCommand(command);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(kLogger, "%s", e.what());
                return false;
            }

            if (!response.success) {
                RCLCPP_WARN(kLogger, "ESP32: %s", response.error_message.c_str());
                return false;
            }

            // Expect "S" followed by one position/velocity pair per joint.
            const char* cursor = response.data.c_str();
            if (*cursor != 'S') {
                RCLCPP_WARN(kLogger, "Unexpected reply from ESP32: '%s'", response.data.c_str());
                return false;
            }
            ++cursor;

            std::vector<double> values(velocities.size() * 2);
            for (double& v : values) {
                char* end = nullptr;
                v = std::strtod(cursor, &end);
                if (end == cursor) {
                    RCLCPP_WARN(kLogger, "Expected %zu values from ESP32, got: '%s'",
                                values.size(), response.data.c_str());
                    return false;
                }
                cursor = end;
            }

            for (size_t i = 0; i < velocities.size(); ++i) {
                received_positions_[i] = values[2 * i];
                received_velocities_[i] = values[2 * i + 1];
            }
            return true;
        }

    } // namespace esp32_motor_hardware_interface
}

PLUGINLIB_EXPORT_CLASS(vc::esp32_motor_hardware_interface::Esp32System, hardware_interface::SystemInterface)
