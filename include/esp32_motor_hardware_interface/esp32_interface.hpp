///
/// \file 				esp32_interface.hpp
/// \author 			Vincent Cadicamo (www.vincentcadicamo.dev) <vcadicamo@icloud.com>
/// \edited             n/a
/// \created			2026-10-07
/// \last-modified		2026-10-07
/// \brief 				Contains the esp32 interface
/// \details
///		See README.md in root dir for more info.

#ifndef ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP
#define ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP

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
        /// \brief      Default constructor
        Esp32Interface();

        /// \brief      Constructor with device path
        /// \param      device      Path to serial device (e.g., "/dev/ttyUSB0")
        /// \param      baud_rate   Communication baud rate (default 115200 for ESP32)
        Esp32Interface(const std::string& device, uint32_t baud_rate = 115200);

        /// \brief      Destructor. Closes connection if still open.
        ~Esp32Interface();

        /// \brief      Opens connection to ESP32
        /// \throws     std::runtime_error if connection fails
        void Connect();

        /// \brief      Closes connection to ESP32
        void Disconnect();

        /// \brief      Sends a raw command to the ESP32
        /// \param      command     The command string to send
        /// \returns    Response from ESP32
        Esp32Response SendCommand(const std::string& command);

        /// \brief      Sends raw binary data to ESP32
        /// \param      data        Binary data to send
        /// \returns    Response from ESP32
        Esp32Response SendBinary(const std::vector<uint8_t>& data);

        /// \brief      Reads available data from ESP32 (non-blocking)
        /// \param      data        Output string to append received data
        /// \returns    Number of bytes read
        uint32_t Read(std::string& data);

        /// \brief      Gets the current connection state
        /// \returns    Current Esp32State
        Esp32State GetState() const;

        /// \brief      Sets communication timeout
        /// \param      timeout_ms  Timeout in milliseconds
        void SetTimeout(int32_t timeout_ms);

        /// \brief      Sets the device path
        /// \param      device      Path to serial device
        void SetDevice(const std::string& device);

    private:
        /// \brief      Internal serial port implementation (could use CppLinuxSerial or custom)
        // SerialPort serial_port_;  // If using CppLinuxSerial

        /// \brief      Device path (e.g., "/dev/ttyUSB0")
        std::string device_;

        /// \brief      Baud rate for communication
        uint32_t baud_rate_;

        /// \brief      Current connection state
        Esp32State state_;

        /// \brief      File descriptor for the serial port
        int file_descriptor_;

        /// \brief      Read timeout in milliseconds
        int32_t timeout_ms_;

        /// \brief      Buffer for reading data
        std::vector<char> read_buffer_;

        /// \brief      Validates that the connection is open
        /// \throws     std::runtime_error if not connected
        void ValidateConnection() const;

        /// \brief      Parses ESP32 response format
        /// \param      raw_data    Raw response from device
        /// \returns    Parsed Esp32Response
        Esp32Response ParseResponse(const std::string& raw_data);
        };

    } // namespace esp32_motor_hardware_interface
}

#endif //ESP32_MOTOR_HARDWARE_INTERFACE_ESP32_INTERFACE_HPP
