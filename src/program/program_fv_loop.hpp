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
        hw_wifi_tx_string("Debug loop entered.");
    } else {
        hw_wifi_tx_string("Armed loop entered.");
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
            // Transmit heavily packed lat/long/alt, flags, battery, and SIV.
            // This is the only time the flight vehicle transmits on 915 Mhz, no command byte is needed.
            //
            // Pack state and flags into a single byte.
            // 4 States can pack into two bits.
            uint8_t packed_flags = 0b00000000;
            // Following bits represent one of each for all relevant flags.
            if (flag_debug)   packed_flags |= 0b00000001;
            if (flag_launch)  packed_flags |= 0b00000010;
            if (flag_burnout) packed_flags |= 0b00000100;
            if (flag_apogee)  packed_flags |= 0b00001000;
            // Following four bytes are GPS, with 16 bits for latitude and longitude each.
            // GPS values are relative to a "center" latitude and longitude and are sent as integers
            // where each increment of "1" represents 1/10^5 degrees, giving a total range of 15/10^5 = 0.32768 degrees or 36km in each direction,
            // along with a resolution of 1/10^5 = 0.00001 degrees or 1.11m.
            uint16_t packed_gps_lat = (uint16_t)((state.latitude - GPS_OFFSET_LAT) * 100000.0);
            uint16_t packed_gps_long = (uint16_t)((state.longitude - GPS_OFFSET_LONG) * 100000.0);
            // Following two bytes are altitude, truncated to 1 meter resolution and fit into 16 bytes.
            // This gives a maximum range of 65536 m.
            //
            // Negative values are assumed to be zero.
            float temp = state.baro_curr;
            if (temp < 0.0) temp = 0.0;
            uint16_t packed_alt = (uint16_t)(temp);
            // Last byte is 4-bit SIV and then 4-bit battey voltage.
            // Battery voltage is such that each increment of "1" represents 0.1 volts starting from 3.2 volts, giving a max of 4.7 volts.
            // SIV is clamped to 0-15, if above 15 then it just stays at 15.
            uint8_t packed_siv = state.siv;
            if (state.siv > 15) packed_siv = 15;
            uint8_t packed_volt = (uint8_t)((hw_bat_volt() - 3.2) * 10.0);
            uint8_t packed_siv_volt = ((packed_siv << 4) & 0b11110000) | (packed_volt & 0b00001111);

            uint8_t buffer[8];
            buffer[0] = packed_flags;
            memcpy(&buffer[1], &packed_gps_lat, 2);
            memcpy(&buffer[3], &packed_gps_long, 2);
            memcpy(&buffer[5], &packed_alt, 2);
            buffer[7] = packed_siv_volt;

            // Attempt transmit of packed data.
            // Does nothing if radio busy.
            hw_lora_transmit(buffer, 8);
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