#pragma once
#include <core/_core.hpp>

/* -------------------------------------------------------------------------- */
/*                                   Driver                                   */
/* -------------------------------------------------------------------------- */
#define PN_I2C_SDA 6
#define PN_I2C_SCL 7

// Global driver struct is used just like on the main board for sensors.
struct CAMI2CConfig {
    uint8_t flag_serial_cmd = CMD_GENERIC_NONE;
    uint32_t buffer_len = 0;
    uint8_t buffer[128];
} driver_cam_i2c_config;

// On CAM board I2C bytes received.
void hw_cam_i2c_receive(int bytes) {
    // Write all bytes to I2C buffer to be processed in bulk later.
    // This makes sure the ISR isn't too long.
    for (size_t i = 0; i < bytes; i++) { driver_cam_i2c_config.buffer[i] = Wire.read(); }
    driver_cam_i2c_config.buffer_len = bytes;
}
// On CAM board I2C bytes requested.
IRAM_ATTR void hw_cam_i2c_request() {
  // Only outputs last written serial command.
  Wire.slaveWrite(&driver_cam_i2c_config.flag_serial_cmd, 1);
}

/* -------------------------------------------------------------------------- */
/*                                    Main                                    */
/* -------------------------------------------------------------------------- */
void program_cam_initialization() {
    // Set up hardware IO.
    Serial.begin(115200);
    Wire.onReceive(hw_cam_i2c_receive);
    Wire.onRequest(hw_cam_i2c_request);
    Wire.begin((uint8_t)I2C_CAM_ADDRESS, PN_I2C_SDA, PN_I2C_SCL, 400000);
}
void program_cam_loop() {
    // If serial data in is available, we read it in.
    // Data in is always assumed to be a single-byte.
    //
    // This single byte is stored as the serial cmd flag,
    // and is outputted whenever a byte is requested from the device (and cleared by I2C below)
    if (Serial.available()) {
        // Read in byte.
        uint8_t cmd = Serial.read();
        driver_cam_i2c_config.flag_serial_cmd = cmd;
        Serial.print("[CAM] Serial CMD: " + String(cmd, HEX) + "\n");
        // Discard remaining bytes.
        while (Serial.available()) { Serial.read(); }
    }

    // I2C Buffer is populated by handler interrupts.
    // First byte represents command and following are contextual.
    if (driver_cam_i2c_config.buffer_len != 0) {
        uint8_t cmd = driver_cam_i2c_config.buffer[0];
        switch (cmd) {
            // Print all following bytes to serial.
            case CMD_CAM_I2C_PRINT: { for (uint32_t i = 1; i < driver_cam_i2c_config.buffer_len; i++) { Serial.print((char)driver_cam_i2c_config.buffer[i]); } break; }
            // Clears any commands which have been held by serial (acknowledging their execution)
            case CMD_CAM_I2C_CLEAR: { driver_cam_i2c_config.flag_serial_cmd = 0x00; Serial.print("[CAM] Serial CMD cleared.\n"); break; }
        }
        // Flush buffer after use.
        driver_cam_i2c_config.buffer_len = 0;
    }
}