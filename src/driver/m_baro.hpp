/* -------------------------------------------------------------------------- */
/*                               driver_baro.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the HP203B, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <driver/m_base.hpp>

/* ---------------------------- State and Config ---------------------------- */
struct BaroConfig { 
    const uint32_t BARO_OSR_4096 = 0b00000000;
    const uint32_t BARO_OSR_2048 = 0b00000100;
    const uint32_t BARO_OSR_1024 = 0b00001000;
    const uint32_t BARO_OSR_512  = 0b00001100;
    const uint32_t BARO_OSR_256  = 0b00010000;
    const uint32_t BARO_OSR_128  = 0b00010100;

    uint32_t osr_setting = BARO_OSR_256;
    float rate_hz = 30.0f;
    float rate_seconds = 1.0f / rate_hz;
    float next_read = 0.0f;
    float bias = 0.0f; 
} hwm_config_baro;

/* ------------------------------- Structures ------------------------------- */
struct BaroData { float altitude; };


/* ------------------------ Interface Implementations ----------------------- */
// No intiailization code for now.
void hwm_baro_init() {}

// Returns if at least one reading is on the sensor based off the time of last
// read and the expected read frequency.
bool hwm_baro_ready() {
    return seconds_us() > hwm_config_baro.next_read;
}

// Return current altitude reading from sensor, halts for conversion time.
BaroData hwm_baro_read() {
    // Altimeter OSR and Channel setting command,
    // followed by altitude read command.
    Wire.beginTransmission(0x77);
    Wire.write(0b01000000 | hwm_config_baro.BARO_OSR_256);
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
    output.altitude += hwm_config_baro.bias;
    
    // Reset ready timer.
    hwm_config_baro.next_read = seconds_us() + hwm_config_baro.rate_seconds;

    // Return.
    return output;
}

// Sets "zero" altitude for the altimeter.
void hwm_baro_set_calibration(float altitude) {
    hwm_config_baro.bias = -altitude;
}