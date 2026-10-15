/* -------------------------------------------------------------------------- */
/*                               driver_cam.hpp                               */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the camera board, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <driver/m_base.hpp>

// Tells camera to trigger video recording if in flight mode
void hwm_cam_trigger() {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_TRIGGER);
  Wire.endTransmission(true);
}

// I2C transmission to tell CAM to clear it's last serial byte.
void hwm_cam_serial_clear() {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_CLEAR);
  Wire.endTransmission(true);
}

// I2C transmission for bytes to be printed over serial.
void hwm_cam_serial_print(String text) {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_PRINT);
  Wire.print(text);
  Wire.endTransmission(true);
}

// I2C Request to CAM always returns last byte over serial.
uint8_t hwm_cam_serial_query() {
  if(Wire.requestFrom(I2C_CAM_ADDRESS, uint8_t(1)) == 0) return CMD_GENERIC_NONE;
  else return Wire.read();
}