/* -------------------------------------------------------------------------- */
/*                               driver_gps.hpp                               */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the SAM-M10Q, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#ifndef MAIN_DRIVER_GPS
#define MAIN_DRIVER_GPS
#include <core.hpp>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <sfe_bus.h>
#include <u-blox_Class_and_ID.h>
#include <u-blox_GNSS.h>
#include <u-blox_config_keys.h>
#include <u-blox_external_typedefs.h>
#include <u-blox_structs.h>

/* ---------------------------------- State --------------------------------- */
struct GPSConfig {
    const float rate_hz = 5.0f;
    const float rate_seconds = 1.0f / rate_hz;

    SFE_UBLOX_GNSS gps_obj;
} gps_config;

/* ------------------------------- Structures ------------------------------- */
struct GPSData {
    float latitude;
    float longitude;
    float altitude;
    uint8_t siv;
};

/* -------------------------- Interface Definitions ------------------------- */
void hw_gps_init();
bool hw_gps_ready();
GPSData hw_gps_read();

/* ------------------------ Interface Implementations ----------------------- */
void hw_gps_init() {
  // Initialize GPS with rate and auto update, force only I2C comms.
  gps_config.gps_obj.begin();
  gps_config.gps_obj.setI2COutput(COM_TYPE_UBX);
  gps_config.gps_obj.setNavigationFrequency((uint32_t)gps_config.rate_hz);
  gps_config.gps_obj.setAutoPVT(true);
}
bool hw_gps_ready() {
  return gps_config.gps_obj.getPVT();
}
GPSData hw_gps_read() {
  // Read directly through library, converting units inline.
  GPSData output;
  output.latitude = (float)gps_config.gps_obj.getLatitude() / 10000000.0;
  output.longitude = (float)gps_config.gps_obj.getLongitude() / 10000000.0;
  output.altitude = (float)gps_config.gps_obj.getAltitude() / 1000.0;
  output.siv = (uint8_t)gps_config.gps_obj.getSIV();
  return output;
}

#endif