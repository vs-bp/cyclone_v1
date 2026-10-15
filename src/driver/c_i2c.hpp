#pragma once
#include <core/_core.hpp>

/* -------------------------------------------------------------------------- */
/*                             CAM MCU I2C Driver                             */
/* -------------------------------------------------------------------------- */
#define PN_CAM_I2C_SDA 6
#define PN_CAM_I2C_SCL 7

// Global driver struct is used just like on the main board for sensors.
struct CAMI2CConfig {
    uint8_t flag_serial_cmd = CMD_GENERIC_NONE;
    uint32_t buffer_len = 0;
    uint8_t buffer[128];
} hwc_i2c_config;

// Internal I2C handlers:
IRAM_ATTR void __i2c_receive(int bytes) {
    // Write all bytes to I2C buffer to be processed in bulk later.
    // This makes sure the ISR isn't too long.
    for (size_t i = 0; i < bytes; i++) { hwc_i2c_config.buffer[i] = Wire.read(); }
    hwc_i2c_config.buffer_len = bytes;
}
IRAM_ATTR void __i2c_request() {
  // Only outputs last written serial command.
  Wire.slaveWrite(&hwc_i2c_config.flag_serial_cmd, 1);
}

// Interface functions.
// Initializes I2C pins and handlers.
void hwc_i2c_init() {
    Wire.onReceive(__i2c_receive);
    Wire.onRequest(__i2c_request);
    Wire.begin((uint8_t)I2C_CAM_ADDRESS, PN_CAM_I2C_SDA, PN_CAM_I2C_SCL, 400000);
}
// Returns byte count in I2C buffer.
uint32_t hwc_i2c_buffer_len() { return hwc_i2c_config.buffer_len; }
// Returns pointer to I2C buffer.
uint8_t* hwc_i2c_buffer() { return hwc_i2c_config.buffer; }
// Empties I2C buffer. (Sets its cursor to zero, doesnt wipe the data)
void hwc_i2c_buffer_flush() { hwc_i2c_config.buffer_len = 0; }
// Sets command byte that is sent on I2C byte reqest to CAM MCU.
void hwc_i2c_set_serial_cmd_flag(uint8_t byte) {
    hwc_i2c_config.flag_serial_cmd = byte;
}
