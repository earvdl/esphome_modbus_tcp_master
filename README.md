# ESPHome Modbus TCP Manager

A robust external component for ESPHome that provides Modbus TCP client functionality with advanced connection management, multiple register support, and optional safety features.

## About this fork
 I (with the extensive help of CoPilot) added mainly two new features:
 - Modbus sensors now support multiple data types:
     - U16 (unsigned int16)
     - S16 (signed int16)
     - U32_BE (unsigned int32, big indian)
     - U32_LE (unsigned in32, little indian)
     - S32_BE (signed int32, big indian)
     - S32_LE (signed in32, little indian)
 - Adaptive caching strategy: Modbus response blocks (multiple registers) are cached for near-future new requests by other sensors. This reduces amount of network traffic. Order of sensors in ESPHome yaml makes a difference now. Order sensors on increasing Modbus address.
 - **Note**: the consequences of the introduction of data types to writing to modbus registers has not been explored yet... Writing was noet tested at all by me (yet). I didn't need it so far.
       
## Features

- 🌐 **Modbus TCP Client** - Connect to any Modbus TCP server/device
- 📊 **Multiple Data Types** - Read 16-bit signed/unsigned integers with scaling
- 🔄 **Read & Write Support** - Single register and multiple register operations
- 🛡️ **Robust Error Handling** - ESP32 stays responsive even when Modbus device is offline
- 📡 **Connection Monitoring** - Real-time connection status reporting
- ⚡ **High-Frequency Polling** - Support for 1-second update intervals
- 🚨 **Optional Watchdog** - Device health monitoring with safe mode
- 🔧 **On-Demand Writes** - Automatic writes triggered by sensor value changes

## Supported Modbus Functions

| Function Code | Description | Usage |
|---------------|-------------|-------|
| 3 | Read Holding Registers | Read/write data registers |
| 4 | Read Input Registers | Read-only input data |
| 6 | Write Single Register | Write individual values |
| 16 | Write Multiple Registers | Bulk write operations |

## Works with ESP32

Tested on ESP32-WROOM-32, esp-idf framework:

```yaml
esp32:
  variant:32
  flash_size: 4MB
  framework:
    type: esp-idf
```

## Installation

Add this to your ESPHome configuration:

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/earvdl/esphome_modbus_tcp_master
      ref: stable
    components: 
      - modbus_tcp_manager
    refresh: 60s
```

## Basic Configuration

### Minimal Setup

```yaml
# Single connection manager for the Modbus device
modbus_tcp_manager:
  id: modbus_device
  host: "192.168.1.100"  # IP address of your Modbus TCP server
  port: 502              # Default Modbus TCP port
  unit_id: 1             # Modbus unit/slave ID

# Connection status monitoring
binary_sensor:
  - platform: modbus_tcp_manager
    modbus_tcp_id: modbus_device
    name: "Modbus Connection Status"
    device_class: connectivity

# Read temperature from input register 0
sensor:
  - platform: modbus_tcp_manager
    modbus_tcp_id: modbus_device
    name: "Temperature Sensor"
    register_address: 0
    function_code: 4      # Read Input Registers
    value_type: U16       # data type is unsigned, 16bits integer
    scale: 0.1            # Multiply raw value by 0.1
    update_interval: 5s
    unit_of_measurement: "°C"
    device_class: "temperature"
    accuracy_decimals: 1
```

## Advanced Configuration

### With Watchdog and Safe Mode

```yaml
modbus_tcp_manager:
  id: modbus_device
  host: "192.168.1.100"
  port: 502
  unit_id: 1
  
  # Optional watchdog - device must increment this register
  watchdog_register: 999
  watchdog_interval: 10s
  
  # Safe mode values written when connection fails
  safe_mode_registers:
    - register: 100    # Setpoint register
      value: 150       # 15.0°C safe setpoint (scaled by 10)
    - register: 101    # Enable register  
      value: 0         # Turn OFF when connection lost
```

### Multiple Sensors with Different Types

```yaml
sensor:
  # Temperature from input register (read-only)
  - platform: modbus_tcp_manager
    modbus_tcp_id: modbus_device
    name: "Inlet Temperature"
    register_address: 0
    function_code: 4      # Input register
    value_type: U16
    scale: 0.1
    update_interval: 2s
    unit_of_measurement: "°C"
    device_class: "temperature"

  # Pressure from holding register (read/write)  
  - platform: modbus_tcp_manager
    modbus_tcp_id: modbus_device
    name: "System Pressure"
    register_address: 10
    function_code: 3      # Holding register
    value_type: U32       # data type is unsigned 32-bits integer, addresses 10 an 11 are read!
    scale: 0.01           # Different scaling
    update_interval: 5s
    unit_of_measurement: "bar"
    device_class: "pressure"

  # Flow rate with offset
  - platform: modbus_tcp_manager
    modbus_tcp_id: modbus_device
    name: "Flow Rate"
    register_address: 20
    function_code: 4
    value_type: U16
    scale: 0.1
    offset: -10.0         # Subtract 10 from scaled value
    update_interval: 3s
    unit_of_measurement: "L/min"
```

## Writing to Modbus Registers

### Manual Control

```yaml
number:
  - platform: template
    name: "Temperature Setpoint"
    id: temp_setpoint
    min_value: 15
    max_value: 25
    step: 0.5
    optimistic: true
    unit_of_measurement: "°C"
    set_action:
      then:
        - lambda: |-
            auto *modbus = id(modbus_device);
            if (modbus != nullptr) {
              int16_t scaled_value = (int16_t)(x * 10);  // Scale to device format
              bool success = modbus->write_register(100, scaled_value);
              if (success) {
                ESP_LOGI("main", "Set temperature setpoint: %.1f°C", x);
              }
            }

switch:
  - platform: template
    name: "System Enable"
    optimistic: true
    turn_on_action:
      - lambda: |-
          auto *modbus = id(modbus_device);
          if (modbus != nullptr) {
            modbus->write_register(101, 1);  // Write 1 to enable
          }
    turn_off_action:
      - lambda: |-
          auto *modbus = id(modbus_device);
          if (modbus != nullptr) {
            modbus->write_register(101, 0);  // Write 0 to disable
          }
```

### Automatic Writes (On-Demand)

```yaml
sensor:
  - platform: dallas
    name: "Local Temperature"
    id: local_temp
    update_interval: 2s
    # Automatically write to Modbus when sensor updates
    on_value:
      then:
        - if:
            condition:
              binary_sensor.is_on: modbus_connection
            then:
              - lambda: |-
                  auto *modbus = id(modbus_device);
                  if (modbus != nullptr && !isnan(x)) {
                    int16_t scaled = (int16_t)(x * 10);
                    modbus->write_register(200, scaled);
                  }
```

### Bulk Operations

```yaml
button:
  - platform: template
    name: "Send All Settings"
    on_press:
      then:
        - lambda: |-
            auto *modbus = id(modbus_device);
            if (modbus != nullptr) {
              // Write multiple registers in one operation
              std::vector<int16_t> values = {
                (int16_t)(id(temp_setpoint).state * 10),  // Setpoint
                (int16_t)(id(local_temp).state * 10),     // Current temp
                id(system_enable).state ? 1 : 0           // Enable status
              };
              bool success = modbus->write_registers(300, values);
              if (success) {
                ESP_LOGI("main", "Bulk write completed - 3 registers");
              }
            }
```

## Configuration Options

### ModbusTCP Manager

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `host` | string | Required | IP address or hostname of Modbus TCP server |
| `port` | int | 502 | Modbus TCP port |
| `unit_id` | int | 1 | Modbus unit/slave ID (1-255) |
| `watchdog_register` | int | Optional | Register for watchdog counter |
| `watchdog_interval` | time | 10s | How often to check watchdog |
| `safe_mode_registers` | list | Optional | Registers to write when connection fails |

### Sensor Platform

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `modbus_tcp_id` | id | Required | Reference to modbus_tcp_manager |
| `register_address` | int | Required | Modbus register address (0-based) |
| `function_code` | int | 3 | 3=Holding registers, 4=Input registers |
| `scale` | float | 0.1 | Multiply raw value by this factor |
| `offset` | float | 0.0 | Add this value after scaling |
| `update_interval` | time | 30s | How often to poll the register |

## Troubleshooting

### Connection Issues

1. **Check network connectivity:**
   ```bash
   ping 192.168.1.100
   telnet 192.168.1.100 502
   ```

2. **Enable debug logging:**
   ```yaml
   logger:
     level: DEBUG
     logs:
       modbus_tcp_manager: DEBUG
   ```

3. **Monitor connection status:**
   - Add the `binary_sensor` for connection monitoring
   - Check Home Assistant for "Modbus Connection Status"

### Common Problems

| Problem | Solution |
|---------|----------|
| Sensor shows "NA" | Check register address, function code, or device availability |
| Values are wrong | Verify `scale` factor and `byte_order` |
| ESP32 crashes when device disconnected | Update to latest component version with improved error handling |
| OpenTherm timing warnings | Increase `update_interval` to 5s or more |
| Slow connection detection | Enable more frequent polling or check network |

### Debug Logs

**Successful operation:**
```
[D][modbus_tcp_manager]: Reading register 0
[D][modbus_tcp_manager]: Register 0: raw=251, scaled=25.10
[I][modbus_tcp_manager]: Successfully wrote value 200 to register 100
```

**Connection problems:**
```
[W][modbus_tcp_manager]: Connection failed to 192.168.1.100:502
[W][modbus_tcp_manager]: Failed to read register 0: Connection failed
```

## Performance Notes

- **Network operations** take 200-400ms - this is normal for TCP
- **Use staggered update intervals** to reduce system load
- **OpenTherm users** should use 5s+ intervals to avoid timing conflicts
- **Memory usage** is minimal (~3KB heap per connection)

## Safety Features

### Connection Monitoring
- Real-time connection status via binary sensor
- Automatic reconnection attempts
- Graceful handling of network failures

### Watchdog System
- Optional register-based device health monitoring
- Configurable check intervals
- Automatic safe mode activation

### Safe Mode
- Automatic activation when connection/watchdog fails
- Configurable register values for safe operation
- Manual recovery when connection restored

## Contributing

Issues and pull requests are welcome! Please ensure:
- Test with real Modbus TCP devices
- Include debug logs for any issues
- Follow ESPHome coding standards

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE.txt) file for details.

## Acknowledgments

- Based on original work by [GiuseppeP96](https://github.com/GiuseppeP96)
- Inspired by [creepystefan/esphome_modbus_tcp](https://github.com/creepystefan/esphome_modbus_tcp)
- Built for the ESPHome and Home Assistant community
