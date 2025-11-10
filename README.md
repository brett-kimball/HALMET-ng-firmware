# HALMET-ng Firmware

Enhanced firmware for [HALMET: Hat Labs Marine Engine & Tank interface](https://shop.hatlabs.fi/products/halmet).

Based on the original [HALMET-example-firmware](https://github.com/hatlabs/HALMET-example-firmware).

To get started, follow the generic SensESP [Getting Started](https://signalk.org/SensESP/pages/getting_started/) instructions.

## Enhancements Over Original Firmware

- **Expanded Analog Inputs**: Support for 8 analog channels (dual ADS1115)
- **Twin Engine Support**: RPM, oil pressure, and temperature monitoring for two engines
- **Other Features**: AIS Gateway, BNO055 compass/attitude, rudder angle, trim tabs, transmission gear
- **Calibration Mode**: Raw sensor value display on web UI and SignalK for setup and troubleshooting
- **SPIFFS Maintenance**: Built-in cleanup tools for configuration management

## Hardware Configuration

**Engine Monitoring (Twin Engines):**
- RPM inputs (D1, D2)
- Oil pressure sensors (A11, A13)
- Coolant temperature sensors (A12, A14)
- Low oil pressure alarms (D3, D4)

**Navigation & Control:**
- Rudder angle (A01)
- Trim tabs (A02)
- Transmission gear (A03,A04)
- BNO055 compass/attitude sensor

**Communications:**
- AIS Gateway (Serial2)
- NMEA 2000 output

## NMEA 2000 PGNs

The firmware transmits the following NMEA 2000 Parameter Group Numbers:

**Engine Monitoring:**
- PGN 127488: Engine Parameters, Rapid Update (RPM)
- PGN 127489: Engine Parameters, Dynamic (oil pressure, temperature, alarms)

**Navigation:**
- PGN 127245: Rudder (rudder angle)
- PGN 127250: Heading (magnetic heading from BNO055)
- PGN 127257: Attitude (pitch and roll from BNO055)

**Vessel Systems:**
- PGN 127493: Transmission Parameters (gear position)
- PGN 130576: Trim Tab Position

**AIS:**
- PGN 129038: Class A Position Report
- PGN 129039: Class B Position Report  
- PGN 129794: Class A Static Data
- PGN 129029: GNSS Position Data
- PGN 130001: AIS Transceiver Status

## Latest Release

[Download firmware.bin](firmware.bin) - Latest compiled version

## Configuration

Edit `src/main.cpp` to customize for your setup. Parts intended to be customized are marked with `EDIT:` comments.

## SPIFFS Maintenance

The firmware includes built-in SPIFFS cleanup functionality for maintenance:

- **Manual cleanup**: Enable `FORCE_SPIFFS_CLEANUP` in `platformio.ini` for file-by-file cleanup
- **Full format**: Enable `FORCE_SPIFFS_FORMAT` in `platformio.ini` for complete SPIFFS wipe

See `docs/cleanup-and-session.md` for detailed instructions. Always backup your configuration before running cleanup operations.
