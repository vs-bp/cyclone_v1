#pragma once
#include <Arduino.h>

/* -------------------------------------------------------------------------- */
/*                                  Keywords                                  */
/* -------------------------------------------------------------------------- */
// Universal command defs.
// Generic cases.
const uint8_t CMD_GENERIC_NONE             = 0x00;
// Ground station internal mode commands.
const uint8_t CMD_SERIAL_OTA_CLEAR         = 0x10;
// Ground station CAM I2C commands.
const uint8_t CMD_CAM_I2C_PRINT            = 0x20;
const uint8_t CMD_CAM_I2C_CLEAR            = 0x21;
const uint8_t CMD_CAM_I2C_MODE_GS          = 0x22;
const uint8_t CMD_CAM_I2C_MODE_FV          = 0x23;
const uint8_t CMD_CAM_I2C_TRIGGER          = 0x24;
// Ground station dispay mode commands.
const uint8_t CMD_DISPLAY_MODE_NONE        = 0x30;
const uint8_t CMD_DISPLAY_MODE_GPS         = 0x31;
const uint8_t CMD_DISPLAY_MODE_ALTITUDE    = 0x32;
const uint8_t CMD_DISPLAY_MODE_EULER       = 0x33;
const uint8_t CMD_DISPLAY_MODE_ACCEL       = 0x34;
const uint8_t CMD_DISPLAY_MODE_GYRO        = 0x35;
const uint8_t CMD_DISPLAY_MODE_CONTROLS    = 0x36;
// WiFi UDP transmission commands.
const uint8_t CMD_WIFI_DATA                = 0xA0;
const uint8_t CMD_WIFI_LOG                 = 0xA1;
const uint8_t CMD_WIFI_OTA_CLEAR_MODE0     = 0xA2;
const uint8_t CMD_WIFI_OTA_CLEAR_MODE1     = 0xA3;
const uint8_t CMD_WIFI_OTA_CLEAR_MODE2     = 0xA4;
const uint8_t CMD_WIFI_OTA_CLEAR_MODE3     = 0xA5;
// Ground station LoRa transmission commands.
const uint8_t CMD_LORA_BREAKLOOP           = 0xC0;
const uint8_t CMD_LORA_HITL_ENABLE         = 0xC1;
const uint8_t CMD_LORA_HITL_DISABLE        = 0xC2;

// Universal I2C address for CAM board on any hardware.
const uint8_t I2C_CAM_ADDRESS = 0x55;

// Universal GPS data transmission settings.
// "Center" point of GPS readings, GPS values are transmitted relative to this
// (but displayed relative to 0,0 lat/long).
//
// Values are also offset from this center by the "radius" to allow convenient transmission
// as a uint.
const float GPS_CENTER_LAT = 27.933083900053056;
const float GPS_CENTER_LONG = -80.70953838981971;
const float GPS_RADIUS = 0.32768;
const float GPS_OFFSET_LAT = GPS_CENTER_LAT - GPS_RADIUS;
const float GPS_OFFSET_LONG = GPS_CENTER_LONG - GPS_RADIUS;