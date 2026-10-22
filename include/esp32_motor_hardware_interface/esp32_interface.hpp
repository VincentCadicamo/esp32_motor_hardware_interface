///
/// \file 				esp32_interface.hpp
/// \author 			Vincent Cadicamo (www.vincentcadicamo.dev) <vcadicamo@icloud.com>
/// \edited             n/a
/// \created			2026-10-07
/// \last-modified		2026-10-22
/// \brief 				Contains the esp32 interface
/// \details
///		UART transport to the ESP32 (e.g. Jetson 40-pin header, /dev/ttyTHS1). Commands are
///		newline-terminated text lines and every command gets exactly one line back.
///		See README.md in root dir for more info.

#ifndef ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP
#define ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace vc {
    namespace esp32_motor_hardware_interface {

        /// \brief      Enumeration of ESP32 communication states
        enum class Esp32State {
            DISCONNECTED,
            CONNECTED,
            ERROR,
        };

        /// \brief      Response structure for ESP32 commands
        struct Esp32Response {
            bool success;
            std::string data;
            std::string error_message;
        };

        /// \brief      Interface class for communicating with ESP32 devices
        class Esp32Interface {

        public:
            /// \brief      Default constructor. Uses "/dev/ttyTHS1" at 115200 baud.
            Esp32Interface();

            /// \brief      Constructor with device path
            /// \param      device      Path to serial device (e.g., "/dev/ttyTHS1")
            /// \param      baud_rate   Communication baud rate (default 115200 for ESP32)
            Esp32Interface(const std::string& device, uint32_t baud_rate = 115200);

            /// \brief      Destructor. Closes connection if still open.
            ~Esp32Interface();

            // Owns a file descriptor, so copying would close it twice.
            Esp32Interface(const Esp32Interface&) = delete;
            Esp32Interface& operator=(const Esp32Interface&) = delete;

            /// \brief      Opens connection to ESP32
            /// \throws     std::runtime_error if connection fails
            void Connect();

            /// \brief      Closes connection to ESP32
            void Disconnect();

            /// \brief      Sends a text command (newline is appended) and waits for one reply line
            /// \param      command     The command string to send
            /// \returns    Response from ESP32
            /// \throws     std::runtime_error if not connected
            Esp32Response SendCommand(const std::string& command);

            /// \brief      Sends raw binary data to ESP32 and waits for one reply line
            /// \param      data        Binary data to send
            /// \returns    Response from ESP32
            /// \throws     std::runtime_error if not connected
            Esp32Response SendBinary(const std::vector<uint8_t>& data);

            /// \brief      Reads available data from ESP32 (non-blocking)
            /// \param      data        Output string to append received data
            /// \returns    Number of bytes read
            uint32_t Read(std::string& data);

            /// \brief      Gets the current connection state
            /// \returns    Current Esp32State
            Esp32State GetState() const;

            /// \brief      Sets the reply timeout used by SendCommand / SendBinary
            /// \param      timeout_ms  Timeout in milliseconds
            void SetTimeout(int32_t timeout_ms);

            /// \brief      Sets the device path. Takes effect on the next Connect().
            /// \param      device      Path to serial device
            void SetDevice(const std::string& device);

        private:
            /// \brief      Device path (e.g., "/dev/ttyTHS1")
            std::string device_;

            /// \brief      Baud rate for communication
            uint32_t baud_rate_;

            /// \brief      Current connection state
            Esp32State state_;

            /// \brief      File descriptor for the serial port
            int file_descriptor_;

            /// \brief      Reply timeout in milliseconds
            int32_t timeout_ms_;

            /// \brief      Scratch buffer for read() calls
            std::vector<char> read_buffer_;

            /// \brief      Received bytes not yet consumed as a full line
            std::string pending_;

            /// \brief      Validates that the connection is open
            /// \throws     std::runtime_error if not connected
            void ValidateConnection() const;

            /// \brief      Discards stale input, writes the bytes, and waits for one reply line
            Esp32Response Transact(const char* bytes, size_t size);

            /// \brief      Writes all bytes, waiting for the port if it is busy
            /// \returns    false on a write error
            bool WriteAll(const char* bytes, size_t size);

            /// \brief      Blocks until a full line arrives or the timeout expires
            /// \param      line        Output line without the trailing newline
            /// \returns    false on timeout or read error
            bool ReadLine(std::string& line);

            /// \brief      Parses ESP32 response format ("ERR <message>" is a failure)
            /// \param      raw_data    Raw response from device
            /// \returns    Parsed Esp32Response
            Esp32Response ParseResponse(const std::string& raw_data);
        };

    } // namespace esp32_motor_hardware_interface
}

#endif //ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP
