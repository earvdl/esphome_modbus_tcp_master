#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <string>
#include <vector>
#include <memory>

#ifdef USE_ESP32
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/inet.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#endif

namespace esphome {
namespace modbus_tcp {

static const char *const TAG = "modbus_tcp_manager";

enum class ModbusFunction : uint8_t {
    READ_COILS = 0x01,
    READ_DISCRETE_INPUTS = 0x02,
    READ_HOLDING_REGISTERS = 0x03,
    READ_INPUT_REGISTERS = 0x04,
    WRITE_SINGLE_COIL = 0x05,
    WRITE_SINGLE_REGISTER = 0x06,
    WRITE_MULTIPLE_COILS = 0x0F,
    WRITE_MULTIPLE_REGISTERS = 0x10
};

struct ModbusResponse {
    bool success;
    std::vector<uint16_t> data;
    std::string error_message;
};

class ModbusTCPManager : public Component {
public:
    ModbusTCPManager(const std::string &host, uint16_t port, uint8_t unit_id)
        : host_(host), port_(port), unit_id_(unit_id),
          is_connected_(false), last_connection_attempt_(0),
          watchdog_register_(0), watchdog_enabled_(false),
          watchdog_interval_(10000), last_watchdog_time_(0),
          watchdog_counter_(0), safe_mode_active_(false),
          connection_check_state_(ConnectionCheckState::IDLE),
          connection_check_sock_(-1), connection_check_start_time_(0),
          connection_check_success_(false),
          persistent_sock_(-1), last_persistent_check_(0) {}

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus TCP Manager for %s:%d", host_.c_str(), port_);
    }

    // Configuration methods
    void set_watchdog_register(uint16_t reg) {
        watchdog_register_ = reg;
        watchdog_enabled_ = true;
        ESP_LOGD(TAG, "Watchdog enabled on register %d", reg);
    }

    void set_watchdog_interval(uint32_t interval) {
        watchdog_interval_ = interval;
    }

    void add_safe_mode_register(uint16_t reg, int16_t value) {
        safe_mode_registers_.push_back({reg, value});
        ESP_LOGD(TAG, "Added safe mode: register %d = %d", reg, value);
    }

    void loop() override {
        uint32_t now = millis();

        // Non-blocking connection health check - keep 5 second interval
        if (now - last_connection_attempt_ > 5000) {
            last_connection_attempt_ = now;
            start_connection_check();  // Start non-blocking check
        }

        // Process connection check state machine (non-blocking, max 5ms per call)
        process_connection_check();

        // Watchdog handling
        if (watchdog_enabled_ && now - last_watchdog_time_ > watchdog_interval_) {
            handle_watchdog();
        }

        // Yield regularly for responsiveness
        if (now % 10 == 0) {
            yield();
        }
    }

    float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

    // Connection status
    bool is_connected() const { return is_connected_; }

    // Force connection status update (used by sensors)
    void mark_connection_failed() {
        is_connected_ = false;
    }

    // Public connection check method (now also non-blocking)
    void check_connection() {
        // If already checking, don't start another
        if (connection_check_state_ != ConnectionCheckState::IDLE) {
            return;
        }
        start_connection_check();
    }

    // Read single register
    ModbusResponse read_register(uint16_t address, ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
        return read_registers(address, 1, function);
    }

    // Read multiple registers
    ModbusResponse read_registers(uint16_t start_address, uint16_t count, ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
        ModbusResponse response;
        response.success = false;

        int sock = create_connection();
        if (sock < 0) {
            response.error_message = "Connection failed";
            is_connected_ = false;
            return response;
        }

        uint16_t request_tid = 0;
        std::vector<uint8_t> request = build_read_request(start_address, count, function, request_tid);

        if (!send_data(sock, request)) {
            response.error_message = "Send failed";
            is_connected_ = false;
            return response;
        }

        std::vector<uint8_t> resp_data = receive_modbus_frame(sock);

        if (resp_data.empty()) {
            response.error_message = "Receive failed";
            is_connected_ = false;
            return response;
        }

        if (!parse_read_response(resp_data, response, function, request_tid)) {
            is_connected_ = false;
            // Protocol mismatch / stream desync: force reconnect on next op
            reset_persistent_socket(sock);
            return response;
        }

        is_connected_ = true;
        response.success = true;
        return response;
    }

    // Write single register
    bool write_register(uint16_t address, int16_t value) {
        ESP_LOGD(TAG, "Writing value %d to register %d", value, address);

        int sock = create_connection();
        if (sock < 0) {
            is_connected_ = false;
            return false;
        }

        uint16_t request_tid = 0;
        std::vector<uint8_t> request = build_write_request(address, value, request_tid);

        bool success = send_data(sock, request);
        if (success) {
            std::vector<uint8_t> response = receive_modbus_frame(sock);
            success = validate_write_response(response, request_tid, 0x06);
        }

        is_connected_ = success;

        if (!success) {
            reset_persistent_socket(sock);
            ESP_LOGW(TAG, "Failed to write to register %d", address);
        } else {
            ESP_LOGD(TAG, "Successfully wrote value %d to register %d", value, address);
        }

        return success;
    }

    // Write multiple registers
    bool write_registers(uint16_t start_address, const std::vector<int16_t>& values) {
        ESP_LOGD(TAG, "Writing %d values starting at register %d", values.size(), start_address);

        if (values.empty() || values.size() > 123) {
            ESP_LOGE(TAG, "Invalid value count: %d", values.size());
            return false;
        }

        int sock = create_connection();
        if (sock < 0) {
            is_connected_ = false;
            return false;
        }

        uint16_t request_tid = 0;
        std::vector<uint8_t> request = build_write_multiple_request(start_address, values, request_tid);

        bool success = send_data(sock, request);
        if (success) {
            std::vector<uint8_t> response = receive_modbus_frame(sock);
            success = validate_write_response(response, request_tid, 0x10);
        }

        is_connected_ = success;

        if (!success) {
            reset_persistent_socket(sock);
            ESP_LOGW(TAG, "Failed to write multiple registers starting at %d", start_address);
        } else {
            ESP_LOGD(TAG, "Successfully wrote %d values starting at register %d", values.size(), start_address);
        }

        return success;
    }

private:
    std::string host_;
    uint16_t port_;
    uint8_t unit_id_;
    bool is_connected_;
    uint32_t last_connection_attempt_;
    uint16_t transaction_id_ = 1;

    // Watchdog variables
    uint16_t watchdog_register_;
    bool watchdog_enabled_;
    uint32_t watchdog_interval_;
    uint32_t last_watchdog_time_;
    uint16_t watchdog_counter_;
    bool safe_mode_active_;

    // Non-blocking connection check state machine
    enum class ConnectionCheckState {
        IDLE,
        CONNECTING,
        CLEANUP
    };
    ConnectionCheckState connection_check_state_;
    int connection_check_sock_;
    uint32_t connection_check_start_time_;
    bool connection_check_success_;  // Track whether the check succeeded

    // Persistent socket pooling
    int persistent_sock_;
    uint32_t last_persistent_check_;

    // Safe mode configuration
    struct SafeModeRegister {
        uint16_t register_addr;
        int16_t value;
    };
    std::vector<SafeModeRegister> safe_mode_registers_;

    // Start non-blocking connection check
    void start_connection_check() {
        if (connection_check_state_ != ConnectionCheckState::IDLE) {
            return;  // Already in progress
        }

        ESP_LOGV(TAG, "Starting non-blocking connection check");
        connection_check_state_ = ConnectionCheckState::CONNECTING;
        connection_check_start_time_ = millis();
        connection_check_success_ = false;

        // Create socket
        connection_check_sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (connection_check_sock_ < 0) {
            ESP_LOGV(TAG, "Could not create socket for connection check");
            connection_check_state_ = ConnectionCheckState::CLEANUP;
            return;
        }

        // Set to non-blocking mode
        int flags = ::fcntl(connection_check_sock_, F_GETFL, 0);
        ::fcntl(connection_check_sock_, F_SETFL, flags | O_NONBLOCK);

        // Start connection attempt
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);
        ::inet_aton(host_.c_str(), &server_addr.sin_addr);

        int result = ::connect(connection_check_sock_, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (result == 0) {
            connection_check_success_ = true;
            connection_check_state_ = ConnectionCheckState::CLEANUP;
        } else if (errno != EINPROGRESS) {
            ESP_LOGV(TAG, "Connection check failed immediately");
            connection_check_state_ = ConnectionCheckState::CLEANUP;
        }
    }

    // Process connection check state machine (max 5ms per call)
    void process_connection_check() {
        uint32_t now = millis();

        switch (connection_check_state_) {
            case ConnectionCheckState::IDLE:
                return;

            case ConnectionCheckState::CONNECTING: {
                fd_set write_fds, error_fds;
                FD_ZERO(&write_fds);
                FD_ZERO(&error_fds);
                FD_SET(connection_check_sock_, &write_fds);
                FD_SET(connection_check_sock_, &error_fds);

                struct timeval timeout;
                timeout.tv_sec = 0;
                timeout.tv_usec = 1000;  // 1ms timeout

                int select_result = ::select(connection_check_sock_ + 1, nullptr, &write_fds, &error_fds, &timeout);

                if (select_result > 0) {
                    if (FD_ISSET(connection_check_sock_, &error_fds)) {
                        ESP_LOGV(TAG, "Connection check failed (error fd set)");
                        connection_check_success_ = false;
                        connection_check_state_ = ConnectionCheckState::CLEANUP;
                    } else if (FD_ISSET(connection_check_sock_, &write_fds)) {
                        int error = 0;
                        socklen_t len = sizeof(error);
                        ::getsockopt(connection_check_sock_, SOL_SOCKET, SO_ERROR, &error, &len);

                        if (error == 0) {
                            ESP_LOGV(TAG, "Connection check succeeded");
                            connection_check_success_ = true;
                        } else {
                            ESP_LOGV(TAG, "Connection check failed (socket error: %d)", error);
                            connection_check_success_ = false;
                        }
                        connection_check_state_ = ConnectionCheckState::CLEANUP;
                    }
                } else if (now - connection_check_start_time_ > 500) {
                    ESP_LOGV(TAG, "Connection check timeout");
                    connection_check_success_ = false;
                    connection_check_state_ = ConnectionCheckState::CLEANUP;
                }
                break;
            }

            case ConnectionCheckState::CLEANUP: {
                if (connection_check_sock_ >= 0) {
                    ::close(connection_check_sock_);
                    connection_check_sock_ = -1;
                }

                if (connection_check_success_) {
                    if (!is_connected_) {
                        ESP_LOGI(TAG, "Modbus connection restored to %s:%d", host_.c_str(), port_);
                        is_connected_ = true;
                    }
                } else {
                    if (is_connected_) {
                        ESP_LOGW(TAG, "Modbus connection lost to %s:%d", host_.c_str(), port_);
                        is_connected_ = false;
                        if (persistent_sock_ >= 0) {
                            ::close(persistent_sock_);
                            persistent_sock_ = -1;
                        }
                    }
                }

                connection_check_state_ = ConnectionCheckState::IDLE;
                break;
            }
        }
    }

    void handle_watchdog() {
        last_watchdog_time_ = millis();

        if (!is_connected_) {
            ESP_LOGW(TAG, "Watchdog: Connection lost, activating safe mode");
            activate_safe_mode();
            return;
        }

        watchdog_counter_++;
        bool write_success = write_register(watchdog_register_, watchdog_counter_);

        if (write_success) {
            ModbusResponse response = read_register(watchdog_register_);

            if (response.success && !response.data.empty()) {
                uint16_t read_value = response.data[0];

                if (read_value == watchdog_counter_) {
                    ESP_LOGD(TAG, "Watchdog OK: wrote %d, read %d - device responding", watchdog_counter_, read_value);

                    if (safe_mode_active_) {
                        ESP_LOGI(TAG, "Watchdog restored, deactivating safe mode");
                        safe_mode_active_ = false;
                    }
                } else {
                    ESP_LOGW(TAG, "Watchdog failed: remote device not responding (wrote %d, read %d)", watchdog_counter_, read_value);
                    activate_safe_mode();
                }
            } else {
                ESP_LOGW(TAG, "Watchdog read failed");
                activate_safe_mode();
            }
        } else {
            ESP_LOGW(TAG, "Watchdog write failed");
            activate_safe_mode();
        }
    }

    void activate_safe_mode() {
        if (safe_mode_active_) return;

        ESP_LOGW(TAG, "Activating safe mode - writing %d safe values", safe_mode_registers_.size());
        safe_mode_active_ = true;

        for (const auto& safe_reg : safe_mode_registers_) {
            write_register(safe_reg.register_addr, safe_reg.value);
            ESP_LOGI(TAG, "Safe mode: Set register %d = %d", safe_reg.register_addr, safe_reg.value);
        }
    }

    void reset_persistent_socket(int sock) {
        if (persistent_sock_ == sock && persistent_sock_ >= 0) {
            ::close(persistent_sock_);
            persistent_sock_ = -1;
        }
    }

    int create_connection() {
        uint32_t now = millis();

        if (persistent_sock_ >= 0 && now - last_persistent_check_ > 5000) {
            last_persistent_check_ = now;

            int error = 0;
            socklen_t len = sizeof(error);
            if (::getsockopt(persistent_sock_, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error != 0) {
                ESP_LOGD(TAG, "Persistent socket validation failed, reconnecting");
                ::close(persistent_sock_);
                persistent_sock_ = -1;
            }
        }

        if (persistent_sock_ >= 0) {
            return persistent_sock_;
        }

        last_persistent_check_ = now;
        int sock = ::socket(AF_INET, SOCK_STREAM, 0);
        if (sock < 0) {
            ESP_LOGV(TAG, "Could not create socket: %d", errno);
            return -1;
        }

        // Slightly less aggressive timeouts to reduce false receive failures
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 150000;  // 150ms
        ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port_);

        if (::inet_aton(host_.c_str(), &server_addr.sin_addr) == 0) {
            struct hostent *he = ::gethostbyname(host_.c_str());
            if (he == nullptr) {
                ESP_LOGV(TAG, "DNS resolution failed: %s", host_.c_str());
                ::close(sock);
                return -1;
            }
            memcpy(&server_addr.sin_addr, he->h_addr, sizeof(server_addr.sin_addr));
        }

        int flags = ::fcntl(sock, F_GETFL, 0);
        ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);

        int connect_result = ::connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (connect_result < 0 && errno == EINPROGRESS) {
            fd_set write_fds;
            FD_ZERO(&write_fds);
            FD_SET(sock, &write_fds);

            struct timeval connect_timeout;
            connect_timeout.tv_sec = 0;
            connect_timeout.tv_usec = 150000;  // 150ms

            int select_result = ::select(sock + 1, nullptr, &write_fds, nullptr, &connect_timeout);
            if (select_result <= 0) {
                ESP_LOGV(TAG, "Connection timeout to %s:%d", host_.c_str(), port_);
                ::close(sock);
                return -1;
            }

            int error = 0;
            socklen_t len = sizeof(error);
            ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
            if (error != 0) {
                ESP_LOGV(TAG, "Connection failed to %s:%d (error: %d)", host_.c_str(), port_, error);
                ::close(sock);
                return -1;
            }
        } else if (connect_result < 0) {
            ESP_LOGV(TAG, "Immediate connection failure to %s:%d", host_.c_str(), port_);
            ::close(sock);
            return -1;
        }

        ::fcntl(sock, F_SETFL, flags);

        persistent_sock_ = sock;
        ESP_LOGD(TAG, "Created persistent socket to %s:%d", host_.c_str(), port_);
        return sock;
    }

    bool send_data(int sock, const std::vector<uint8_t>& data) {
        int sent = ::send(sock, data.data(), data.size(), 0);
        if (sent != (int)data.size()) {
            ESP_LOGV(TAG, "Send failed: %d/%d bytes", sent, data.size());
            reset_persistent_socket(sock);
            return false;
        }
        return true;
    }

    bool recv_exact(int sock, uint8_t *buf, size_t len) {
        size_t offset = 0;
        while (offset < len) {
            int r = ::recv(sock, buf + offset, len - offset, 0);
            if (r <= 0) {
                reset_persistent_socket(sock);
                return false;
            }
            offset += static_cast<size_t>(r);
        }
        return true;
    }

    std::vector<uint8_t> receive_modbus_frame(int sock) {
        // MBAP header = 7 bytes: TID(2), PID(2), LEN(2), UID(1)
        uint8_t mbap[7];
        if (!recv_exact(sock, mbap, sizeof(mbap))) {
            return {};
        }

        uint16_t protocol_id = (static_cast<uint16_t>(mbap[2]) << 8) | mbap[3];
        uint16_t length = (static_cast<uint16_t>(mbap[4]) << 8) | mbap[5];

        // Length counts UID+PDU. Must be at least 2 bytes (UID + FC)
        // Max practical Modbus TCP ADU is 260 bytes => length <= 254.
        if (protocol_id != 0 || length < 2 || length > 254) {
            ESP_LOGW(TAG, "Invalid MBAP header: pid=%u len=%u", protocol_id, length);
            reset_persistent_socket(sock);
            return {};
        }

        std::vector<uint8_t> frame(6 + length);  // first 6 + (uid+pdu)
        memcpy(frame.data(), mbap, 6);

        // Need to copy UID (mbap[6]) + rest(length-1) from socket
        frame[6] = mbap[6];
        if (length > 1) {
            if (!recv_exact(sock, frame.data() + 7, length - 1)) {
                return {};
            }
        }

        return frame;
    }

    std::vector<uint8_t> build_read_request(uint16_t address, uint16_t count, ModbusFunction function, uint16_t &request_tid) {
        request_tid = transaction_id_++;
        return {
            static_cast<uint8_t>((request_tid >> 8) & 0xFF),
            static_cast<uint8_t>(request_tid & 0xFF),
            0x00, 0x00,
            0x00, 0x06,
            unit_id_,
            static_cast<uint8_t>(function),
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((count >> 8) & 0xFF),
            static_cast<uint8_t>(count & 0xFF)
        };
    }

    std::vector<uint8_t> build_write_request(uint16_t address, int16_t value, uint16_t &request_tid) {
        request_tid = transaction_id_++;
        return {
            static_cast<uint8_t>((request_tid >> 8) & 0xFF),
            static_cast<uint8_t>(request_tid & 0xFF),
            0x00, 0x00,
            0x00, 0x06,
            unit_id_,
            0x06,
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF)
        };
    }

    std::vector<uint8_t> build_write_multiple_request(uint16_t address, const std::vector<int16_t>& values, uint16_t &request_tid) {
        uint16_t count = values.size();
        uint8_t byte_count = count * 2;
        request_tid = transaction_id_++;

        std::vector<uint8_t> request = {
            static_cast<uint8_t>((request_tid >> 8) & 0xFF),
            static_cast<uint8_t>(request_tid & 0xFF),
            0x00, 0x00,
            static_cast<uint8_t>(((7 + byte_count) >> 8) & 0xFF),
            static_cast<uint8_t>((7 + byte_count) & 0xFF),
            unit_id_,
            0x10,
            static_cast<uint8_t>((address >> 8) & 0xFF),
            static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((count >> 8) & 0xFF),
            static_cast<uint8_t>(count & 0xFF),
            byte_count
        };

        for (int16_t value : values) {
            request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
            request.push_back(static_cast<uint8_t>(value & 0xFF));
        }

        return request;
    }

    bool validate_write_response(const std::vector<uint8_t>& data, uint16_t expected_tid, uint8_t expected_function) {
        if (data.size() < 8) {
            ESP_LOGW(TAG, "Write response too short");
            return false;
        }

        uint16_t tid = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        uint16_t pid = (static_cast<uint16_t>(data[2]) << 8) | data[3];
        uint16_t len = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        uint8_t uid = data[6];
        uint8_t fc = data[7];

        if (tid != expected_tid || pid != 0 || uid != unit_id_) {
            ESP_LOGW(TAG, "Write response header mismatch (tid=%u/%u pid=%u uid=%u/%u)",
                     tid, expected_tid, pid, uid, unit_id_);
            return false;
        }

        if (fc == static_cast<uint8_t>(expected_function | 0x80)) {
            uint8_t ex = (data.size() > 8) ? data[8] : 0xFF;
            ESP_LOGW(TAG, "Write response Modbus exception: %u", ex);
            return false;
        }

        if (fc != expected_function) {
            ESP_LOGW(TAG, "Write response invalid function: got=%u expected=%u", fc, expected_function);
            return false;
        }

        // Basic ADU length consistency: total bytes should be 6 + LEN
        if (data.size() != static_cast<size_t>(6 + len)) {
            ESP_LOGW(TAG, "Write response length mismatch: got=%u expected=%u", (unsigned)data.size(), (unsigned)(6 + len));
            return false;
        }

        return true;
    }

    bool parse_read_response(const std::vector<uint8_t>& data, ModbusResponse& response, ModbusFunction function, uint16_t expected_tid) {
        if (data.size() < 9) {
            response.error_message = "Response too short";
            return false;
        }

        uint16_t tid = (static_cast<uint16_t>(data[0]) << 8) | data[1];
        uint16_t pid = (static_cast<uint16_t>(data[2]) << 8) | data[3];
        uint16_t len = (static_cast<uint16_t>(data[4]) << 8) | data[5];
        uint8_t uid = data[6];
        uint8_t fc = data[7];
        uint8_t expected_fc = static_cast<uint8_t>(function);

        if (tid != expected_tid) {
            response.error_message = "Transaction ID mismatch";
            return false;
        }

        if (pid != 0) {
            response.error_message = "Invalid protocol ID";
            return false;
        }

        if (uid != unit_id_) {
            response.error_message = "Unit ID mismatch";
            return false;
        }

        if (data.size() != static_cast<size_t>(6 + len)) {
            response.error_message = "Response length mismatch";
            return false;
        }

        if (fc == static_cast<uint8_t>(expected_fc | 0x80)) {
            uint8_t ex = (data.size() > 8) ? data[8] : 0xFF;
            response.error_message = "Modbus exception code " + to_string(ex);
            return false;
        }

        if (fc != expected_fc) {
            response.error_message = "Invalid function code";
            return false;
        }

        uint8_t byte_count = data[8];
        if ((byte_count % 2) != 0) {
            response.error_message = "Invalid byte count";
            return false;
        }

        if (data.size() < static_cast<size_t>(9 + byte_count)) {
            response.error_message = "Incomplete response";
            return false;
        }

        response.data.clear();
        for (uint8_t i = 0; i < byte_count; i += 2) {
            uint16_t value = (static_cast<uint16_t>(data[9 + i]) << 8) | data[9 + i + 1];
            response.data.push_back(value);
        }

        return true;
    }
};

// Sensor class
class ModbusTCPSensor : public PollingComponent, public sensor::Sensor {
public:
    ModbusTCPSensor(ModbusTCPManager *parent, uint16_t register_address,
                    uint8_t function_code, float scale, float offset, uint32_t update_interval)
        : parent_(parent), register_address_(register_address),
          function_code_(function_code), scale_(scale), offset_(offset) {
        this->set_update_interval(update_interval);
    }

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus sensor for register %d", register_address_);
    }

    void update() override {
        if (!parent_->is_connected()) {
            ESP_LOGV(TAG, "Triggering connection check for register %d", register_address_);
            const_cast<ModbusTCPManager*>(parent_)->check_connection();

            if (!parent_->is_connected()) {
                ESP_LOGV(TAG, "Modbus still not connected, skipping update");
                return;
            }
        }

        ModbusFunction func = (function_code_ == 4) ?
            ModbusFunction::READ_INPUT_REGISTERS :
            ModbusFunction::READ_HOLDING_REGISTERS;

        ESP_LOGD(TAG, "Reading register %d", register_address_);
        ModbusResponse response = parent_->read_register(register_address_, func);

        if (response.success && !response.data.empty()) {
            int16_t raw_value = static_cast<int16_t>(response.data[0]);
            float scaled_value = (raw_value * scale_) + offset_;

            ESP_LOGD(TAG, "Register %d: raw=%d, scaled=%.2f", register_address_, raw_value, scaled_value);
            this->publish_state(scaled_value);
        } else {
            ESP_LOGW(TAG, "Failed to read register %d: %s", register_address_, response.error_message.c_str());
            const_cast<ModbusTCPManager*>(parent_)->mark_connection_failed();
        }

        yield();
    }

private:
    ModbusTCPManager *parent_;
    uint16_t register_address_;
    uint8_t function_code_;
    float scale_;
    float offset_;
};

// Connection status sensor
class ModbusTCPConnectionSensor : public PollingComponent, public binary_sensor::BinarySensor {
public:
    ModbusTCPConnectionSensor(ModbusTCPManager *parent) : parent_(parent) {
        this->set_update_interval(1000);  // Check every 1 second for faster response
    }

    void setup() override {
        ESP_LOGD(TAG, "Setting up Modbus connection status sensor");
    }

    void update() override {
        bool connected = parent_->is_connected();
        this->publish_state(connected);
        ESP_LOGV(TAG, "Modbus connection status: %s", connected ? "Connected" : "Disconnected");
    }

private:
    ModbusTCPManager *parent_;
};

}  // namespace modbus_tcp
}  // namespace esphome
