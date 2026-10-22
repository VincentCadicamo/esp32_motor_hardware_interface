///
/// \file 				esp32_interface.cpp
/// \author 			Vincent Cadicamo (www.vincentcadicamo.dev) <vcadicamo@icloud.com>
/// \edited             n/a
/// \created			2026-10-22
/// \last-modified		2026-10-22
/// \brief 				POSIX termios implementation of the esp32 interface
/// \details
///		See README.md in root dir for more info.

#include "esp32_motor_hardware_interface/esp32_interface.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace vc {
    namespace esp32_motor_hardware_interface {

        namespace {

            speed_t ToSpeed(uint32_t baud_rate) {
                switch (baud_rate) {
                    case 9600:   return B9600;
                    case 19200:  return B19200;
                    case 38400:  return B38400;
                    case 57600:  return B57600;
                    case 115200: return B115200;
                    case 230400: return B230400;
                    default:
                        throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
                }
            }

        } // namespace

        Esp32Interface::Esp32Interface() : Esp32Interface("/dev/ttyTHS1") {}

        Esp32Interface::Esp32Interface(const std::string& device, uint32_t baud_rate)
            : device_(device),
              baud_rate_(baud_rate),
              state_(Esp32State::DISCONNECTED),
              file_descriptor_(-1),
              timeout_ms_(100),
              read_buffer_(256) {}

        Esp32Interface::~Esp32Interface() {
            Disconnect();
        }

        void Esp32Interface::Connect() {
            if (state_ == Esp32State::CONNECTED) {
                return;
            }

            const speed_t speed = ToSpeed(baud_rate_);

            file_descriptor_ = open(device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (file_descriptor_ < 0) {
                state_ = Esp32State::ERROR;
                throw std::runtime_error("Failed to open " + device_ + ": " + std::strerror(errno));
            }

            termios tty{};
            if (tcgetattr(file_descriptor_, &tty) != 0) {
                const std::string error = std::strerror(errno);
                Disconnect();
                state_ = Esp32State::ERROR;
                throw std::runtime_error("tcgetattr failed on " + device_ + ": " + error);
            }

            // 8N1, raw bytes, no flow control, no echo
            cfmakeraw(&tty);
            cfsetispeed(&tty, speed);
            cfsetospeed(&tty, speed);
            tty.c_cflag |= (CLOCAL | CREAD);
            tty.c_cflag &= ~(CSTOPB | CRTSCTS);
            tty.c_cc[VMIN] = 0;
            tty.c_cc[VTIME] = 0;

            if (tcsetattr(file_descriptor_, TCSANOW, &tty) != 0) {
                const std::string error = std::strerror(errno);
                Disconnect();
                state_ = Esp32State::ERROR;
                throw std::runtime_error("tcsetattr failed on " + device_ + ": " + error);
            }

            tcflush(file_descriptor_, TCIOFLUSH);
            pending_.clear();
            state_ = Esp32State::CONNECTED;
        }

        void Esp32Interface::Disconnect() {
            if (file_descriptor_ >= 0) {
                close(file_descriptor_);
                file_descriptor_ = -1;
            }
            pending_.clear();
            state_ = Esp32State::DISCONNECTED;
        }

        Esp32Response Esp32Interface::SendCommand(const std::string& command) {
            ValidateConnection();
            const std::string line = command + "\n";
            return Transact(line.data(), line.size());
        }

        Esp32Response Esp32Interface::SendBinary(const std::vector<uint8_t>& data) {
            ValidateConnection();
            return Transact(reinterpret_cast<const char*>(data.data()), data.size());
        }

        uint32_t Esp32Interface::Read(std::string& data) {
            if (state_ != Esp32State::CONNECTED) {
                return 0;
            }

            uint32_t total = 0;
            while (true) {
                const ssize_t n = read(file_descriptor_, read_buffer_.data(), read_buffer_.size());
                if (n > 0) {
                    data.append(read_buffer_.data(), static_cast<size_t>(n));
                    total += static_cast<uint32_t>(n);
                    continue;
                }
                if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                    state_ = Esp32State::ERROR;
                }
                return total;
            }
        }

        Esp32State Esp32Interface::GetState() const {
            return state_;
        }

        void Esp32Interface::SetTimeout(int32_t timeout_ms) {
            timeout_ms_ = timeout_ms;
        }

        void Esp32Interface::SetDevice(const std::string& device) {
            device_ = device;
        }

        void Esp32Interface::ValidateConnection() const {
            if (state_ != Esp32State::CONNECTED) {
                throw std::runtime_error("ESP32 on " + device_ + " is not connected");
            }
        }

        Esp32Response Esp32Interface::Transact(const char* bytes, size_t size) {
            // Drop anything left over (e.g. a reply that arrived after a previous timeout),
            // otherwise every later reply would be matched to the wrong command.
            tcflush(file_descriptor_, TCIFLUSH);
            pending_.clear();

            if (!WriteAll(bytes, size)) {
                state_ = Esp32State::ERROR;
                return {false, "", "write to " + device_ + " failed: " + std::strerror(errno)};
            }

            std::string line;
            if (!ReadLine(line)) {
                return {false, "", "no reply from ESP32 within " + std::to_string(timeout_ms_) + " ms"};
            }
            return ParseResponse(line);
        }

        bool Esp32Interface::WriteAll(const char* bytes, size_t size) {
            size_t written = 0;
            while (written < size) {
                const ssize_t n = write(file_descriptor_, bytes + written, size - written);
                if (n > 0) {
                    written += static_cast<size_t>(n);
                } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                    pollfd pfd{file_descriptor_, POLLOUT, 0};
                    if (poll(&pfd, 1, timeout_ms_) <= 0) {
                        return false;
                    }
                } else if (n < 0 && errno != EINTR) {
                    return false;
                }
            }
            return true;
        }

        bool Esp32Interface::ReadLine(std::string& line) {
            using Clock = std::chrono::steady_clock;
            const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms_);

            while (true) {
                const size_t newline = pending_.find('\n');
                if (newline != std::string::npos) {
                    line = pending_.substr(0, newline);
                    pending_.erase(0, newline + 1);
                    if (!line.empty() && line.back() == '\r') {
                        line.pop_back();
                    }
                    return true;
                }

                const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                    deadline - Clock::now()).count();
                if (remaining <= 0) {
                    return false;
                }

                pollfd pfd{file_descriptor_, POLLIN, 0};
                const int ready = poll(&pfd, 1, static_cast<int>(remaining));
                if (ready < 0 && errno != EINTR) {
                    state_ = Esp32State::ERROR;
                    return false;
                }
                if (ready > 0) {
                    Read(pending_);
                }
            }
        }

        Esp32Response Esp32Interface::ParseResponse(const std::string& raw_data) {
            if (raw_data.rfind("ERR", 0) == 0) {
                const std::string message = raw_data.size() > 4 ? raw_data.substr(4) : "unknown error";
                return {false, raw_data, message};
            }
            return {true, raw_data, ""};
        }

    } // namespace esp32_motor_hardware_interface
}