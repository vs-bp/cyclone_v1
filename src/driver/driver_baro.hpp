/* -------------------------------------------------------------------------- */
/*                               driver_baro.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the HP203B, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* ---------------------------- State and Config ---------------------------- */
struct BaroConfig { 
    const uint32_t BARO_OSR_4096 = 0b00000000;
    const uint32_t BARO_OSR_2048 = 0b00000100;
    const uint32_t BARO_OSR_1024 = 0b00001000;
    const uint32_t BARO_OSR_512  = 0b00001100;
    const uint32_t BARO_OSR_256  = 0b00010000;
    const uint32_t BARO_OSR_128  = 0b00010100;

    uint32_t osr_setting = BARO_OSR_256;
    float rate_hz = 1.0f / 50.0f;
    float rate_seconds = 1.0f / rate_hz;
    float next_read = 0.0f;
    float bias = 0.0f; 
} driver_baro_config;

/* ------------------------------- Structures ------------------------------- */
struct BaroData { float altitude; };


/* ------------------------ Interface Implementations ----------------------- */
// No intiailization code for now.
void hw_baro_init() {}

// Returns if at least one reading is on the sensor based off the time of last
// read and the expected read frequency.
bool hw_baro_ready() {
    return seconds_us() > driver_baro_config.next_read;
}

// Return current altitude reading from sensor, halts for conversion time.
BaroData hw_baro_read() {
    // Altimeter OSR and Channel setting command,
    // followed by altitude read command.
    Wire.beginTransmission(0x77);
    Wire.write(0b01000000 | driver_baro_config.BARO_OSR_256);
    Wire.endTransmission(true);
    Wire.beginTransmission(0x77);
    Wire.write(0x31);
    Wire.endTransmission(true);


    // Collect altitude data from I2C and apply bias.
    Wire.requestFrom(0x77, 3);
    unsigned int data[3];
    data[0] = Wire.read(); 
    data[1] = Wire.read(); 
    data[2] = Wire.read();
    BaroData output;
    output.altitude = (float)((data[0] & 0x0F)*65536 + (data[1]*256) + data[2]) / 100.0f;
    output.altitude += driver_baro_config.bias;
    
    // Reset ready timer.
    driver_baro_config.next_read = seconds_us() + driver_baro_config.rate_seconds;

    // Return.
    return output;
}

// Sets "zero" altitude for the altimeter.
void hw_baro_set_calibration(float altitude) {
    driver_baro_config.bias = -altitude;
}