```
# ESP32 BLE GATT Server

BLE GATT server implementation for ESP32 using NimBLE stack. Exposes sensor data (temperature, humidity, voltage, current, power, battery state) over three standard BLE services.

## Features

- **Environmental Sensing Service (0x181A)**: Temperature and humidity characteristics
- **Power Service (0x181F)**: Voltage, current, and power characteristics  
- **Battery Service (0x180F)**: State of charge and runtime characteristics
- Read and notify support on all characteristics
- Secure connections with bonding/pairing
- Automatic reconnection and advertising management

## Hardware Requirements

- ESP32 (any variant with BLE support)
- ESP-IDF v4.4+ (tested on v5.x)

## Project Structure

```



├── main.cpp           # Application entry point
├── ble.hpp            # BLE interface header
├── ble.cpp            # BLE implementation
└── CMakeLists.txt     # Component registration
```

## Building

```bash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Usage

The server initializes automatically on boot and begins advertising as "Test". Connect with any BLE client (nRF Connect, LightBlue, etc.) to read characteristics or subscribe to notifications.

### Data Format

All sensor values are transmitted as `int16_t` with an implied exponent of -2:
- Transmitted value: `sensor_value * 100`
- Actual value: `transmitted_value / 100`

Example: Temperature of 35.42°C → transmitted as `3542`

### API

```cpp
ble::init()    // Initialize BLE stack and NVS
ble::start()   // Begin advertising
ble::stop()    // Stop advertising
ble::deinit()  // Cleanup and shutdown
```

## Configuration

Edit `ble.cpp` to modify:

- **Device name**: `BLE_GAP_NAME` (line 164)
- **Sensor functions**: `get_temperature()`, `get_humidity()`, etc. (lines 154-168)
- **Security settings**: `ble_hs_cfg` security parameters (lines 565-580)
- **Debug logging**: `BLE_DEBUG` macro (line 24)

## Security

Configured for:
- Bonding enabled
- Secure Connections (LE Secure Connections)
- MITM protection
- No I/O capabilities (auto-pairing)

Keys are stored in NVS flash for persistent bonding across reboots.

## Services & Characteristics

| Service | UUID | Characteristic | UUID | Properties |
|---------|------|----------------|------|------------|
| Environmental Sensing | 0x181A | Temperature | 0x2A6E | Read, Notify |
| Environmental Sensing | 0x181A | Humidity | 0x2A6F | Read, Notify |
| Power | 0x181F | Voltage | 0x2B18 | Read, Notify |
| Power | 0x181F | Current | 0x2AEE | Read, Notify |
| Power | 0x181F | Power | 0x2B05 | Read, Notify |
| Battery | 0x180F | State of Charge | 0x2A19 | Read, Notify |
| Battery | 0x180F | Runtime | 0x2B2E | Read, Notify |

## Notes

- Current implementation returns mock sensor data via `constexpr` functions
- Replace getter functions with actual sensor reads for production use
- NVS initialization can be controlled via `NVS_ALREADY_INITIALIZED` macro
- Notifications are sent automatically when clients subscribe to characteristics

## License

MIT License
```