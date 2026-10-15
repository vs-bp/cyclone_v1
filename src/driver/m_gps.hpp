/* -------------------------------------------------------------------------- */
/*                               driver_gps.hpp                               */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the SAM-M10Q, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <driver/m_base.hpp>

/* ---------------------------------- State --------------------------------- */
struct GPSConfig {
    const float rate_hz = 5.0f;
    const float rate_seconds = 1.0f / rate_hz;

    SFE_UBLOX_GNSS gps_obj;
} hwm_config_gps;

/* ------------------------------- Structures ------------------------------- */
struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    uint8_t siv;
};

/* -------------------------------- Interface ------------------------------- */
// Initialize GPS with rate and auto update, force only I2C comms.
void hwm_gps_init() {
  hwm_config_gps.gps_obj.begin();
  hwm_config_gps.gps_obj.setI2COutput(COM_TYPE_UBX);
  hwm_config_gps.gps_obj.setNavigationFrequency((uint32_t)hwm_config_gps.rate_hz);
  hwm_config_gps.gps_obj.setAutoPVT(true);
}

bool hwm_gps_ready() {
  return hwm_config_gps.gps_obj.getPVT();
}

GPSData hwm_gps_read() {
  // Read directly through library, converting units inline.
  GPSData output;
  output.latitude = (float)hwm_config_gps.gps_obj.getLatitude() / 10000000.0;
  output.longitude = (float)hwm_config_gps.gps_obj.getLongitude() / 10000000.0;
  output.altitude = (float)hwm_config_gps.gps_obj.getAltitude() / 1000.0;
  output.siv = (uint8_t)hwm_config_gps.gps_obj.getSIV();
  return output;
}