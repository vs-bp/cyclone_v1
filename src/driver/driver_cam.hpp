/* -------------------------------------------------------------------------- */
/*                               driver_cam.hpp                               */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the camera board, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

// True or false tells camera whether it's on the
// flight vehicle or ground station.
void hw_cam_init(bool fv) {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  if (fv) Wire.write(CMD_CAM_I2C_MODE_FV);
  else Wire.write(CMD_CAM_I2C_MODE_GS);
  Wire.endTransmission(true);
}

// Tells camera to trigger video recording if in flight mode.
// Does nothing in GS mode.
void hw_cam_trigger() {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_TRIGGER);
  Wire.endTransmission(true);
}

// I2C transmission to tell CAM to clear it's last serial byte.
void hw_cam_serial_clear() {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_CLEAR);
  Wire.endTransmission(true);
}

// I2C transmission for bytes to be printed over serial.
void hw_cam_serial_print(String text) {
  Wire.beginTransmission(I2C_CAM_ADDRESS);
  Wire.write(CMD_CAM_I2C_PRINT);
  Wire.print(text);
  Wire.endTransmission(true);
}

// I2C Request to CAM always returns last byte over serial.
uint8_t hw_cam_serial_query() {
  if(Wire.requestFrom(I2C_CAM_ADDRESS, 1) == 0) return CMD_GENERIC_NONE;
  else return Wire.read();
}