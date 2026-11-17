# ESP_SAE_V1

A compact ESP32-S3 project that reads orientation data from an ICM-20948 inertial measurement unit (IMU) and broadcasts it over Bluetooth Low Energy (BLE) as JSON.

This project is designed for real-time motion tracking and can be used by an Android app, a desktop client, or any BLE-compatible receiver that wants roll, pitch, and yaw values from a compact embedded sensor.

## Overview

The system does the following:

- Initializes an ESP32-S3 board
- Detects and configures the ICM-20948 sensor over I2C
- Calibrates the gyroscope to remove offset drift
- Computes orientation using a complementary filter
- Sends the data as a BLE notification

The sketch outputs a message in the following format:

```json
{"r":12.45,"p":-3.21,"y":48.90}
```

Where:

- `r` = roll (degrees)
- `p` = pitch (degrees)
- `y` = yaw (degrees)

## Hardware

- ESP32-S3 board
- ICM-20948 IMU module
- Jumper wires
- 3.3V power source

## Wiring

The firmware is configured for the following pins:

- SDA = GPIO 20
- SCL = GPIO 21
- ICM-20948 I2C address = `0x69`

Typical wiring:

- IMU SDA -> ESP32-S3 GPIO 20
- IMU SCL -> ESP32-S3 GPIO 21
- IMU VCC -> 3.3V
- IMU GND -> GND

## Features

- I2C communication with the ICM-20948
- WHO_AM_I validation to confirm sensor detection
- Gyroscope bias calibration
- Accelerometer-based tilt detection
- Gyroscope integration for angular velocity tracking
- Complementary filter for stable orientation estimation
- BLE service advertising with notifications
- Real-time JSON output at about 50 Hz

## BLE Configuration

The device is configured with:

- Device name: `ESP32-S3-MATHIS`
- BLE service UUID: `12345678-1234-1234-1234-123456789000`
- BLE characteristic UUID: `12345678-1234-1234-1234-123456789001`

The ESP32 exposes a BLE server and pushes notifications when at least one client is connected.

## Required Libraries

Use the Arduino IDE or PlatformIO with these dependencies:

- ESP32 board support
- NimBLE Arduino library

For Arduino IDE:

1. Install the ESP32 board package in Boards Manager.
2. Install the NimBLE library used by this project.
3. Select the correct ESP32-S3 board.

## Software Setup

### Arduino IDE

1. Open the sketch.
2. Select your ESP32-S3 board.
3. Set the correct serial port.
4. Upload the sketch.

### PlatformIO (optional)

If using PlatformIO, create a project for the ESP32-S3 and add the required library dependencies for NimBLE.

## Usage

After uploading:

1. Power the ESP32.
2. Open the serial monitor at `115200`.
3. Observe sensor initialization and calibration logs.
4. Connect to the device over BLE using a compatible client.
5. Read the notifications or subscribe to the characteristic.

## Data Interpretation

The firmware computes orientation in degrees. The values are updated continuously and represent the estimated attitude of the sensor:

- Roll: rotation around the X axis
- Pitch: rotation around the Y axis
- Yaw: rotation around the Z axis

## Project Structure

This project is intentionally simple and consists of a single Arduino sketch:

- `index.html` — main firmware code for ESP32-S3, ICM-20948 sensor, and BLE communication

## Notes

- The ICM-20948 configuration in this code assumes a standard 3.3V-powered module.
- Sensor calibration should be performed when the device is stationary.
- The complementary filter is a lightweight and effective approach for orientation estimation on embedded devices.
- BLE notifications are sent every 20 ms, which is roughly 50 Hz.

## License

This project is provided as-is for educational and prototyping use. Please check any local licensing or hardware-specific constraints before using it in production.

## Author

This project was created as an ESP32 + IMU + BLE orientation prototype.

## Example Serial Output

```text
ESP32-S3 + ICM-20948 + BLE
Initialisation ICM-20948...
WHO_AM_I : 0xEA
ICM-20948 detecte !
Calibration gyroscope...
Ne pas bouger le capteur !
Calibration terminee !
BLE actif !
ESP32-S3-MATHIS
{"r":0.00,"p":0.00,"y":0.00}
```

## Future Improvements

Possible next steps for the project include:

- Magnetometer fusion for better yaw stability
- Kalman filter implementation
- Sensor fusion with quaternions
- Android or Web BLE client integration
- Data logging to SD card or flash memory
- Battery monitoring

## Troubleshooting

If the device does not initialize:

- Verify I2C wiring and power
- Confirm the chip is an ICM-20948
- Check the serial output for communication errors
- Ensure the correct board package and library are installed

If BLE is not visible:

- Confirm the ESP32-S3 supports BLE advertising
- Check if the BLE service is started correctly
- Verify the serial monitor shows `BLE actif !`

## Summary

`ESP_SAE_V1` is a practical prototype for controlling and monitoring sensor orientation using ESP32-S3 and an ICM-20948 IMU over BLE. It demonstrates embedded sensor acquisition, calibration, filtering, and wireless transmission in a compact and easy-to-extend design.
