#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */
void program_fv_loop(ProgramStateFV& state, bool flag_debug) {
    /* ----------------------------- Initialization ----------------------------- */
    // Conditional disabling of wifi if intending to arm for flight.
    // Wifi logging of current loop entered.
    if (flag_debug) {
        hw_wifi_tx_log("Debug loop entered.");
    } else {
        hw_wifi_tx_log("Armed loop entered.");
        hw_wifi_off();
    }

    // Flag states for in-flight events.
    // Flags fixed in debug.
    bool flag_launch = false;
    bool flag_burnout = false;
    bool flag_apogee = false;
    if (flag_debug) {
        flag_launch = true;
        flag_burnout = true;
        flag_apogee = false;
    }

    /* --------------------------- Sensor Calibration --------------------------- */
    // TODO.

    /* -------------------------------- Main Loop ------------------------------- */
    while (true) {
        // Guidance, navigation, and control from baro and IMU.
        if (hw_baro_ready()) {
            // Data read resets timers.
            BaroData data_baro = hw_baro_read();
            state.baro_rate = (state.baro_rate - state.baro_curr) / baro_config.rate_seconds;
            state.baro_curr = data_baro.altitude;

            IMUDataFIFO data_imu = hw_imu_integrate(state.q);
            state.ab = data_imu.ab;
            state.wb = data_imu.wb;
            state.dcmbe = data_imu.dcmbe;
            state.dcmeb = data_imu.dcmeb;
            state.q = data_imu.q1;
            state.eul_rate = (data_imu.eul - state.eul_curr) / baro_config.rate_seconds;
            state.eul_curr = data_imu.eul;

            // Complementary filter estimate.
            // Note, assumes ab = ae.
            state.cpl_velocity = state.cpl_velocity_factor*(state.cpl_velocity + state.ab.x * baro_config.rate_seconds) + (1.0f - state.cpl_velocity_factor)*state.baro_rate;
            state.cpl_altitude = state.cpl_altitude_factor*(state.cpl_altitude + state.cpl_velocity * baro_config.rate_seconds) + (1.0f - state.cpl_altitude_factor)*state.baro_curr;

            // Kalman filter estimate.

            // Controls (from complementary estimate)
            // TODO.
        }

        // GPS data read and immediate attempt at LoRa transmission.
        if (hw_gps_ready()) {
            // Data read.
            GPSData data_gps = hw_gps_read();
            state.latitude = data_gps.latitude;
            state.longitude = data_gps.longitude;
            state.siv = data_gps.siv;

            // Pack data.
            

            // Attempt transmit of packed data.
            // Does nothing if radio busy.
            hw_lora_transmit_buffer(buffer, 8);
        }

        // TODO Wifi TX.

        // LoRa flag handling.
        if (hw_lora_transmit_action_complete()) {
            // TODO.
        }
        if (hw_lora_receive_action_complete()) {
            // TODO.
        }
        // TODO.

        // Event handling.
        // TODO.
        
    }    
}