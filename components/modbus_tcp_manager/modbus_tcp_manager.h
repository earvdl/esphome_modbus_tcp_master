#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <string>
#include <vector>
#include <memory>
#include <cmath>

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

enum class ModbusValueType : uint8_t {
  U16,
  S16,
  U32_BE,  // reg N high, reg N+1 low
  S32_BE,
  U32_LE,  // reg N low, reg N+1 high
  S32_LE
};

struct ModbusResponse {
  bool success;
  std::vector<uint16_t> data;
  std::string error_message;
};

class ModbusTCPManager : public Component {
 public:
  ModbusTCPManager(const std::string &host, uint16_t port, uint8_t unit_id)
      : host_(host),
        port_(port),
        unit_id_(unit_id),
        is_connected_(false),
        last_connection_attempt_(0),
        watchdog_register_(0),
        watchdog_enabled_(false),
        watchdog_interval_(10000),
        last_watchdog_time_(0),
        watchdog_counter_(0),
        safe_mode_active_(false),
        connection_check_state_(ConnectionCheckState::IDLE),
        connection_check_sock_(-1),
        connection_check_start_time_(0),
        connection_check_success_(false),
        persistent_sock_(-1),
        last_persistent_check_(0),
        last_reconnect_ms_(0),
        reconnect_cooldown_ms_(200){}

  void setup() override {
    ESP_LOGD(TAG, "Setting up Modbus TCP Manager for %s:%d", host_.c_str(), port_);
  }

  void set_watchdog_register(uint16_t reg) {
    watchdog_register_ = reg;
    watchdog_enabled_ = true;
    ESP_LOGD(TAG, "Watchdog enabled on register %d", reg);
  }

  void set_watchdog_interval(uint32_t interval) { watchdog_interval_ = interval; }

  void add_safe_mode_register(uint16_t reg, int16_t value) {
    safe_mode_registers_.push_back({reg, value});
    ESP_LOGD(TAG, "Added safe mode: register %d = %d", reg, value);
  }

  void loop() override {
    uint32_t now = millis();

    if (now - last_connection_attempt_ > 15000) {
      last_connection_attempt_ = now;
      start_connection_check();
    }

    process_connection_check();

    if (watchdog_enabled_ && now - last_watchdog_time_ > watchdog_interval_) {
      handle_watchdog();
    }

    if (now % 10 == 0) {
      yield();
    }
  }

  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool is_connected() const { return is_connected_; }

  bool in_reconnect_cooldown() const {
    return is_connected_ && (millis() - last_reconnect_ms_ < reconnect_cooldown_ms_);
  }

  void mark_connection_failed() { is_connected_ = false; }

  void check_connection() {
    if (connection_check_state_ != ConnectionCheckState::IDLE) return;
    start_connection_check();
  }

  ModbusResponse read_register(uint16_t address,
                               ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
    return read_registers(address, 1, function);
  }

  ModbusResponse read_registers(uint16_t start_address, uint16_t count,
                                ModbusFunction function = ModbusFunction::READ_HOLDING_REGISTERS) {
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
      // One retry using a fresh socket (helps with transient Wi-Fi/TCP hiccups)
      reset_persistent_socket(sock);
    
      int retry_sock = create_connection();
      if (retry_sock >= 0) {
        if (send_data(retry_sock, request)) {
          resp_data = receive_modbus_frame(retry_sock);
        }
      }
    }

    if (resp_data.empty()) {
      response.error_message = "Receive failed";
      is_connected_ = false;
      invalidate_register_cache();
      return response;
    }

    if (!parse_read_response(resp_data, response, function, request_tid)) {
      is_connected_ = false;
      reset_persistent_socket(sock);
      invalidate_register_cache();
      return response;
    }

    is_connected_ = true;
    response.success = true;
    return response;
  }

  void invalidate_register_cache() {
    for (size_t i = 0; i < CACHE_SIZE; i++) reg_cache_[i].valid = false;
  }
  
  ModbusResponse read_registers_cached(uint16_t start_reg, uint16_t count, ModbusFunction function_code, uint32_t ttl_ms) {
  // Caching/fetching strategy (general-purpose):
  //
  // 1) Cache lookup order
  //    a) Exact hit: same (function_code, start_reg, count) within ttl_ms.
  //    b) Range hit: requested [start_reg, start_reg+count) fully contained in a
  //       cached block for the same function_code and still within ttl_ms.
  //       Return a sliced subset of cached words.
  //
  // 2) Miss handling (adaptive expansion)
  //    - On miss, perform a wider read around the requested range to exploit spatial locality.
  //    - Expanded block size is bounded (e.g. 8..32 registers) and aligned (e.g. 4-register boundary)
  //      to make future nearby requests likely range-hits.
  //    - Store expanded response as one cache entry, then retry cache lookup for original request.
  //    - If expanded read fails or still cannot satisfy request, fallback to direct exact read.
  //
  // 3) Storage policy
  //    - Fixed-size ring cache (CACHE_SIZE entries), each entry stores one full Modbus response block.
  //    - New entries overwrite oldest (round-robin).
  //    - Entry key: (function_code, start_reg, count), plus timestamp and response payload.
  //
  // 4) Coherency and safety
  //    - Cache is invalidated on connection loss/socket reset/read failure.
  //    - Cache is separated by Modbus function code (do not mix FC03/FC04 data).
  //    - ttl_ms is caller-controlled: short TTL for fast-changing values, longer TTL for slower values.
  //
  // 5) Tuning notes
  //    - CACHE_SIZE controls number of cached blocks, not number of registers.
  //    - Larger expanded blocks improve hit rate for clustered addresses but increase bus payload.
  //    - Too-small TTL reduces hit rate; too-large TTL can serve stale values.
    
    auto try_from_cache = [&](uint16_t req_start, uint16_t req_count) -> ModbusResponse {
      const uint32_t tnow = millis();  // IMPORTANT: fresh time per lookup
  
      // 1) Exact hit
      for (size_t i = 0; i < CACHE_SIZE; i++) {
        auto &e = reg_cache_[i];
        if (!e.valid) continue;
        if (e.function_code != function_code) continue;
        if (!e.response.success) continue;
      
        const uint32_t age_ms = tnow - e.ts_ms;
        if (age_ms > ttl_ms) continue;
      
        if (e.start_reg == req_start && e.count == req_count) {
          ESP_LOGD(TAG, "CACHE HIT exact fc=%u start=0x%04X count=%u age=%ums slot=%u",
                   (unsigned) function_code, (unsigned) req_start, (unsigned) req_count,
                   (unsigned) age_ms, (unsigned) i);
          return e.response;
        }
      }
  
      // 2) Range hit
      for (size_t i = 0; i < CACHE_SIZE; i++) {
        auto &e = reg_cache_[i];
        if (!e.valid) continue;
        if (e.function_code != function_code) continue;
        if (!e.response.success) continue;
  
        const uint32_t age_ms = tnow - e.ts_ms;
        if (age_ms > ttl_ms) continue;
  
        const uint32_t req_start_u = req_start;
        const uint32_t req_end_excl = req_start_u + req_count;
        const uint32_t blk_start = e.start_reg;
        const uint32_t blk_end_excl = blk_start + e.count;
  
        if (req_start_u >= blk_start && req_end_excl <= blk_end_excl) {
          const size_t word_offset = static_cast<size_t>(req_start_u - blk_start);
          const size_t need_words = static_cast<size_t>(req_count);
  
          if (e.response.data.size() >= word_offset + need_words) {
            ModbusResponse out;
            out.success = true;
            out.error_message.clear();
            out.data.assign(e.response.data.begin() + word_offset,
                            e.response.data.begin() + word_offset + need_words);
  
            ESP_LOGD(TAG,
                     "CACHE HIT range fc=%u req=0x%04X/%u from block=0x%04X/%u age=%ums slot=%u",
                     (unsigned) function_code, (unsigned) req_start, (unsigned) req_count,
                     (unsigned) e.start_reg, (unsigned) e.count, (unsigned) age_ms, (unsigned) i);
            return out;
          }
        }
      }
  
      ModbusResponse miss;
      miss.success = false;
      miss.error_message = "cache miss";
      return miss;
    };
  
    auto store_cache = [&](uint16_t s, uint16_t c, const ModbusResponse &resp) {
      // Dedup refresh: if same key exists, refresh in place
      for (size_t i = 0; i < CACHE_SIZE; i++) {
        auto &e = reg_cache_[i];
        if (!e.valid) continue;
        if (e.function_code == function_code && e.start_reg == s && e.count == c) {
          e.ts_ms = millis();
          e.response = resp;
          return;
        }
      }
  
      // Otherwise round-robin insert
      auto &slot = reg_cache_[reg_cache_next_];
      slot.valid = true;
      slot.function_code = function_code;
      slot.start_reg = s;
      slot.count = c;
      slot.ts_ms = millis();
      slot.response = resp;
      reg_cache_next_ = (reg_cache_next_ + 1) % CACHE_SIZE;
    };
  
    // A) Try cache first
    ModbusResponse cached = try_from_cache(start_reg, count);
    if (cached.success) return cached;
  
    // B) Adaptive expansion
    constexpr uint16_t MIN_EXPAND = 8;
    constexpr uint16_t MAX_EXPAND = 32;
    constexpr uint16_t ALIGN = 4;
  
    uint16_t expand_count = count * 4;
    if (expand_count < MIN_EXPAND) expand_count = MIN_EXPAND;
    if (expand_count > MAX_EXPAND) expand_count = MAX_EXPAND;
  
    uint16_t expand_start = static_cast<uint16_t>((start_reg / ALIGN) * ALIGN);
  
    const uint32_t req_end = static_cast<uint32_t>(start_reg) + count;
    uint32_t exp_end = static_cast<uint32_t>(expand_start) + expand_count;
    if (req_end > exp_end) {
      uint32_t needed = req_end - expand_start;
      expand_count = static_cast<uint16_t>(needed > MAX_EXPAND ? MAX_EXPAND : needed);
      exp_end = static_cast<uint32_t>(expand_start) + expand_count;
      if (req_end > exp_end) {
        uint32_t shift = req_end - exp_end;
        expand_start = static_cast<uint16_t>((shift > expand_start) ? 0 : (expand_start - shift));
      }
    }
  
    if (expand_count < count) expand_count = count;
    if (expand_count > MAX_EXPAND) expand_count = MAX_EXPAND;
  
    ESP_LOGD(TAG, "CACHE MISS fc=%u req=0x%04X/%u -> EXPAND 0x%04X/%u",
             (unsigned) function_code, (unsigned) start_reg, (unsigned) count,
             (unsigned) expand_start, (unsigned) expand_count);
  
    // C) Expanded read
    ModbusResponse expanded = this->read_registers(expand_start, expand_count, function_code);
    if (expanded.success) {
      store_cache(expand_start, expand_count, expanded);
  
      // immediate retry should now hit
      ModbusResponse post = try_from_cache(start_reg, count);
      if (post.success) return post;
  
      ESP_LOGW(TAG, "Post-expand lookup still missed fc=%u req=0x%04X/%u block=0x%04X/%u",
               (unsigned) function_code, (unsigned) start_reg, (unsigned) count,
               (unsigned) expand_start, (unsigned) expand_count);
    } else {
      ESP_LOGW(TAG, "Expanded read failed fc=%u start=0x%04X count=%u: %s",
               (unsigned) function_code, (unsigned) expand_start, (unsigned) expand_count,
               expanded.error_message.c_str());
    }
  
    // D) Fallback direct
    ESP_LOGD(TAG, "CACHE MISS fallback direct fc=%u start=0x%04X count=%u",
             (unsigned) function_code, (unsigned) start_reg, (unsigned) count);
    
    ModbusResponse direct = this->read_registers(start_reg, count, function_code);
    
    // Only cache successful reads to avoid sticky cached failures
    if (direct.success) {
      store_cache(start_reg, count, direct);
    }
    
    return direct;
  }  


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
      invalidate_register_cache();
      ESP_LOGW(TAG, "Failed to write to register %d", address);
    } else {
      // conservative: any write may affect read cache view
      invalidate_register_cache();
      ESP_LOGD(TAG, "Successfully wrote value %d to register %d", value, address);
    }
    return success;
  }

  bool write_registers(uint16_t start_address, const std::vector<int16_t> &values) {
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
      invalidate_register_cache();
      ESP_LOGW(TAG, "Failed to write multiple registers starting at %d", start_address);
    } else {
      // conservative: clear cached reads after successful write
      invalidate_register_cache();
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

  uint16_t watchdog_register_;
  bool watchdog_enabled_;
  uint32_t watchdog_interval_;

  uint32_t last_reconnect_ms_;
  uint32_t reconnect_cooldown_ms_;

  uint32_t last_watchdog_time_;
  uint16_t watchdog_counter_;
  bool safe_mode_active_;

  //--------------------------------------------
  // type and field for modbus read caching
  struct RegisterCacheEntry {
    bool valid{false};
    ModbusFunction function_code{ModbusFunction::READ_HOLDING_REGISTERS};
    uint16_t start_reg{0};
    uint16_t count{0};
    uint32_t ts_ms{0};
    ModbusResponse response;
  };
  
  static constexpr size_t CACHE_SIZE = 8;
  RegisterCacheEntry reg_cache_[CACHE_SIZE];
  size_t reg_cache_next_{0};

  //--------------------------------------------

  enum class ConnectionCheckState { IDLE, CONNECTING, CLEANUP };
  ConnectionCheckState connection_check_state_;
  int connection_check_sock_;
  uint32_t connection_check_start_time_;
  bool connection_check_success_;

  int persistent_sock_;
  uint32_t last_persistent_check_;

  struct SafeModeRegister {
    uint16_t register_addr;
    int16_t value;
  };
  std::vector<SafeModeRegister> safe_mode_registers_;

  void start_connection_check() {
    if (connection_check_state_ != ConnectionCheckState::IDLE) return;

    connection_check_state_ = ConnectionCheckState::CONNECTING;
    connection_check_start_time_ = millis();
    connection_check_success_ = false;

    connection_check_sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (connection_check_sock_ < 0) {
      connection_check_state_ = ConnectionCheckState::CLEANUP;
      return;
    }

    int flags = ::fcntl(connection_check_sock_, F_GETFL, 0);
    ::fcntl(connection_check_sock_, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    ::inet_aton(host_.c_str(), &server_addr.sin_addr);

    int result = ::connect(connection_check_sock_, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (result == 0) {
      connection_check_success_ = true;
      connection_check_state_ = ConnectionCheckState::CLEANUP;
    } else if (errno != EINPROGRESS) {
      connection_check_state_ = ConnectionCheckState::CLEANUP;
    }
  }

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

        struct timeval timeout{0, 1000};
        int select_result =
            ::select(connection_check_sock_ + 1, nullptr, &write_fds, &error_fds, &timeout);

        if (select_result > 0) {
          if (FD_ISSET(connection_check_sock_, &error_fds)) {
            connection_check_success_ = false;
            connection_check_state_ = ConnectionCheckState::CLEANUP;
          } else if (FD_ISSET(connection_check_sock_, &write_fds)) {
            int error = 0;
            socklen_t len = sizeof(error);
            ::getsockopt(connection_check_sock_, SOL_SOCKET, SO_ERROR, &error, &len);
            connection_check_success_ = (error == 0);
            connection_check_state_ = ConnectionCheckState::CLEANUP;
          }
        } else if (now - connection_check_start_time_ > 500) {
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
            last_reconnect_ms_ = millis();
            invalidate_register_cache();
          }
         
        } else {
          if (is_connected_) {
            ESP_LOGW(TAG, "Modbus connection lost to %s:%d", host_.c_str(), port_);
            is_connected_ = false;
            invalidate_register_cache();
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
          if (safe_mode_active_) {
            ESP_LOGI(TAG, "Watchdog restored, deactivating safe mode");
            safe_mode_active_ = false;
          }
        } else {
          ESP_LOGW(TAG, "Watchdog failed: remote device not responding (wrote %d, read %d)",
                   watchdog_counter_, read_value);
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
    for (const auto &safe_reg : safe_mode_registers_) {
      write_register(safe_reg.register_addr, safe_reg.value);
    }
  }

  void reset_persistent_socket(int sock) {
    if (persistent_sock_ == sock && persistent_sock_ >= 0) {
      ::close(persistent_sock_);
      invalidate_register_cache();
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
        ::close(persistent_sock_);
        persistent_sock_ = -1;
      }
    }

    if (persistent_sock_ >= 0) return persistent_sock_;

    last_persistent_check_ = now;
    int sock = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return -1;

    struct timeval timeout{0, 350000};
    ::setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);

    if (::inet_aton(host_.c_str(), &server_addr.sin_addr) == 0) {
      struct hostent *he = ::gethostbyname(host_.c_str());
      if (he == nullptr) {
        ::close(sock);
        return -1;
      }
      memcpy(&server_addr.sin_addr, he->h_addr, sizeof(server_addr.sin_addr));
    }

    int flags = ::fcntl(sock, F_GETFL, 0);
    ::fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    int connect_result = ::connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (connect_result < 0 && errno == EINPROGRESS) {
      fd_set write_fds;
      FD_ZERO(&write_fds);
      FD_SET(sock, &write_fds);

      struct timeval connect_timeout{0, 350000};
      int select_result = ::select(sock + 1, nullptr, &write_fds, nullptr, &connect_timeout);
      if (select_result <= 0) {
        ::close(sock);
        return -1;
      }

      int error = 0;
      socklen_t len = sizeof(error);
      ::getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
      if (error != 0) {
        ::close(sock);
        return -1;
      }
    } else if (connect_result < 0) {
      ::close(sock);
      return -1;
    }

    ::fcntl(sock, F_SETFL, flags);
    persistent_sock_ = sock;
    ESP_LOGD(TAG, "Created persistent socket to %s:%d", host_.c_str(), port_);
    return sock;
  }

  bool send_data(int sock, const std::vector<uint8_t> &data) {
    size_t offset = 0;
    while (offset < data.size()) {
      int sent = ::send(sock, data.data() + offset, data.size() - offset, 0);
      if (sent <= 0) {
        reset_persistent_socket(sock);
        invalidate_register_cache();
        return false;
      }
      offset += static_cast<size_t>(sent);
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
    uint8_t mbap[7];
    if (!recv_exact(sock, mbap, sizeof(mbap))) return {};

    uint16_t protocol_id = (static_cast<uint16_t>(mbap[2]) << 8) | mbap[3];
    uint16_t length = (static_cast<uint16_t>(mbap[4]) << 8) | mbap[5];

    if (protocol_id != 0 || length < 2 || length > 254) {
      reset_persistent_socket(sock);
      return {};
    }

    std::vector<uint8_t> frame(6 + length);
    memcpy(frame.data(), mbap, 6);
    frame[6] = mbap[6];

    if (length > 1) {
      if (!recv_exact(sock, frame.data() + 7, length - 1)) return {};
    }

    return frame;
  }

  std::vector<uint8_t> build_read_request(uint16_t address, uint16_t count, ModbusFunction function,
                                          uint16_t &request_tid) {
    request_tid = transaction_id_++;
    return {static_cast<uint8_t>((request_tid >> 8) & 0xFF), static_cast<uint8_t>(request_tid & 0xFF),
            0x00, 0x00, 0x00, 0x06, unit_id_, static_cast<uint8_t>(function),
            static_cast<uint8_t>((address >> 8) & 0xFF), static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((count >> 8) & 0xFF), static_cast<uint8_t>(count & 0xFF)};
  }

  std::vector<uint8_t> build_write_request(uint16_t address, int16_t value, uint16_t &request_tid) {
    request_tid = transaction_id_++;
    return {static_cast<uint8_t>((request_tid >> 8) & 0xFF), static_cast<uint8_t>(request_tid & 0xFF),
            0x00, 0x00, 0x00, 0x06, unit_id_, 0x06,
            static_cast<uint8_t>((address >> 8) & 0xFF), static_cast<uint8_t>(address & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF)};
  }

  std::vector<uint8_t> build_write_multiple_request(uint16_t address, const std::vector<int16_t> &values,
                                                    uint16_t &request_tid) {
    uint16_t count = values.size();
    uint8_t byte_count = count * 2;
    request_tid = transaction_id_++;

    std::vector<uint8_t> request = {
        static_cast<uint8_t>((request_tid >> 8) & 0xFF), static_cast<uint8_t>(request_tid & 0xFF),
        0x00, 0x00, static_cast<uint8_t>(((7 + byte_count) >> 8) & 0xFF),
        static_cast<uint8_t>((7 + byte_count) & 0xFF), unit_id_, 0x10,
        static_cast<uint8_t>((address >> 8) & 0xFF), static_cast<uint8_t>(address & 0xFF),
        static_cast<uint8_t>((count >> 8) & 0xFF), static_cast<uint8_t>(count & 0xFF), byte_count};

    for (int16_t value : values) {
      request.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
      request.push_back(static_cast<uint8_t>(value & 0xFF));
    }

    return request;
  }

  bool validate_write_response(const std::vector<uint8_t> &data, uint16_t expected_tid,
                               uint8_t expected_function) {
    if (data.size() < 8) return false;

    uint16_t tid = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    uint16_t pid = (static_cast<uint16_t>(data[2]) << 8) | data[3];
    uint16_t len = (static_cast<uint16_t>(data[4]) << 8) | data[5];
    uint8_t uid = data[6];
    uint8_t fc = data[7];

    if (tid != expected_tid || pid != 0 || uid != unit_id_) return false;
    if (fc == static_cast<uint8_t>(expected_function | 0x80)) return false;
    if (fc != expected_function) return false;
    if (data.size() != static_cast<size_t>(6 + len)) return false;

    return true;
  }

  bool parse_read_response(const std::vector<uint8_t> &data, ModbusResponse &response,
                           ModbusFunction function, uint16_t expected_tid) {
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

// Existing single-register sensor
class ModbusTCPSensor : public PollingComponent, public sensor::Sensor {
 public:
  ModbusTCPSensor(ModbusTCPManager *parent, uint16_t register_address, uint8_t function_code, float scale,
                  float offset, uint32_t update_interval)
      : parent_(parent),
        register_address_(register_address),
        function_code_(function_code),
        scale_(scale),
        offset_(offset) {
    this->set_update_interval(update_interval);
  }

  void setup() override { ESP_LOGD(TAG, "Setting up Modbus sensor for register %d", register_address_); }

  void update() override {
    if (!parent_->is_connected()) {
      const_cast<ModbusTCPManager *>(parent_)->check_connection();
      if (!parent_->is_connected()) return;
    }

    if (parent_->in_reconnect_cooldown()) {
      ESP_LOGV(TAG, "Reconnect cooldown active, skipping poll for reg %u", register_address_);
      return;
    }

    ModbusFunction func = (function_code_ == 4) ? ModbusFunction::READ_INPUT_REGISTERS
                                                : ModbusFunction::READ_HOLDING_REGISTERS;

    ModbusResponse response = parent_->read_registers_cached(register_address_, 1, func, 1500);
    if (response.success && !response.data.empty()) {
      int16_t raw_value = static_cast<int16_t>(response.data[0]);
      float scaled_value = (raw_value * scale_) + offset_;
      this->publish_state(scaled_value);
    } else {
      ESP_LOGW(TAG, "Failed to read register %d: %s", register_address_, response.error_message.c_str());
      const_cast<ModbusTCPManager *>(parent_)->mark_connection_failed();
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

// New advanced sensor: atomic 16/32-bit read from one transaction
class ModbusTCPAdvancedSensor : public PollingComponent, public sensor::Sensor {
 public:
  ModbusTCPAdvancedSensor(ModbusTCPManager *parent, uint16_t register_address, uint8_t function_code,
                          float scale, float offset, uint32_t update_interval, ModbusValueType value_type)
      : parent_(parent),
        register_address_(register_address),
        function_code_(function_code),
        scale_(scale),
        offset_(offset),
        value_type_(value_type) {
    this->set_update_interval(update_interval);
  }

  void setup() override {
    ESP_LOGD(TAG, "Setting up Modbus advanced sensor for register %d (type=%d)", register_address_,
             static_cast<int>(value_type_));
  }

  void update() override {
    if (!parent_->is_connected()) {
      const_cast<ModbusTCPManager *>(parent_)->check_connection();
      if (!parent_->is_connected()) return;
    }

    if (parent_->in_reconnect_cooldown()) {
      ESP_LOGV(TAG, "Reconnect cooldown active, skipping poll for reg %u", register_address_);
      return;
    }

    ModbusFunction func = (function_code_ == 4) ? ModbusFunction::READ_INPUT_REGISTERS
                                                : ModbusFunction::READ_HOLDING_REGISTERS;

    const uint16_t reg_count = (value_type_ == ModbusValueType::U32_BE || value_type_ == ModbusValueType::S32_BE ||
                                value_type_ == ModbusValueType::U32_LE || value_type_ == ModbusValueType::S32_LE)
                                   ? 2
                                   : 1;

    ModbusResponse response = parent_->read_registers_cached(register_address_, reg_count, func, 1500);
    if (!response.success) {
      ESP_LOGW(TAG, "Failed to read register %d (count=%d): %s", register_address_, reg_count,
               response.error_message.c_str());
      const_cast<ModbusTCPManager *>(parent_)->mark_connection_failed();
      return;
    }
    if (response.data.size() < reg_count) {
      ESP_LOGW(TAG, "Short response for register %d", register_address_);
      const_cast<ModbusTCPManager *>(parent_)->mark_connection_failed();
      return;
    }

    float out = NAN;
    switch (value_type_) {
      case ModbusValueType::U16: {
        uint16_t v = response.data[0];
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
      case ModbusValueType::S16: {
        int16_t v = static_cast<int16_t>(response.data[0]);
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
      case ModbusValueType::U32_BE: {
        uint32_t hi = static_cast<uint32_t>(response.data[0]);
        uint32_t lo = static_cast<uint32_t>(response.data[1]);
        uint32_t v = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF);
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
      case ModbusValueType::S32_BE: {
        uint32_t hi = static_cast<uint32_t>(response.data[0]);
        uint32_t lo = static_cast<uint32_t>(response.data[1]);
        uint32_t raw = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF);
        int32_t v = static_cast<int32_t>(raw);
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
      case ModbusValueType::U32_LE: {
        uint32_t lo = static_cast<uint32_t>(response.data[0]);
        uint32_t hi = static_cast<uint32_t>(response.data[1]);
        uint32_t v = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF);
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
      case ModbusValueType::S32_LE: {
        uint32_t lo = static_cast<uint32_t>(response.data[0]);
        uint32_t hi = static_cast<uint32_t>(response.data[1]);
        uint32_t raw = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF);
        int32_t v = static_cast<int32_t>(raw);
        out = (static_cast<float>(v) * scale_) + offset_;
        break;
      }
    }

    if (!std::isnan(out) && !std::isinf(out)) {
      this->publish_state(out);
    }
    yield();
  }

 private:
  ModbusTCPManager *parent_;
  uint16_t register_address_;
  uint8_t function_code_;
  float scale_;
  float offset_;
  ModbusValueType value_type_;
};

class ModbusTCPConnectionSensor : public PollingComponent, public binary_sensor::BinarySensor {
 public:
  explicit ModbusTCPConnectionSensor(ModbusTCPManager *parent) : parent_(parent) {
    this->set_update_interval(1000);
  }

  void setup() override { ESP_LOGD(TAG, "Setting up Modbus connection status sensor"); }

  void update() override { this->publish_state(parent_->is_connected()); }

 private:
  ModbusTCPManager *parent_;
};

}  // namespace modbus_tcp
}  // namespace esphome
