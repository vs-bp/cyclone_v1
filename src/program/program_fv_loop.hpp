#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */
void program_fv_loop(ProgramStateFV& state, bool debug) {
    state.flag_debug = debug;

    /* ----------------------------- Initialization ----------------------------- */
    // Conditional disabling of wifi if intending to arm for flight.
    // Wifi logging of current loop entered.
    if (state.flag_debug) {
        hw_wifi_tx_log("Debug loop entered.");
    } else {
        hw_wifi_tx_log("Armed loop entered.");
        hw_wifi_off();
    }

    /* ---------------------------- Flags, Thresholds, and Timers ---------------------------- */
    // Timer for wifi-logging when in debug state.
    float wifi_tx_next = 0.0f;
    const float wifi_tx_rate = 1.0f / 25.0f;

    const float threshold_pyro_fire_time = 5.0f;
    const float threshold_main_deploy = 200.0f;
    const float threshold_launch_detect_gs = 3.0f;
    const float threshold_burnout_time = 1.5f;
    const float threshold_min_apogee_time = 10.0f;
    const float threshold_max_apogee_time = 15.0f;

    float timer_launch = -1.0f;

    // Flag states for in-flight events.
    // Flags fixed in debug.
    if (state.flag_debug) {
        state.flag_launch = true;
        state.flag_burnout = true;
        state.flag_apogee = false;
    } else {
        state.flag_launch = false;
        state.flag_burnout = false;
        state.flag_apogee = false;
    }

    /* --------------------------- Sensor Calibration --------------------------- */
    // Calibrate gyros and altimeter by averaging readings.
    // Accelerometer calibration isn't really required so we dont do it.
    hw_wifi_tx_log("Starting calibration.");
    const float calibration_time = 30.0f;
    float calibration_end = seconds_us() + calibration_time;
    BaroData data_baro_accumulator;
    IMUDataFIFO data_imu_accumulator;
    uint32_t baro_read_count = 0;
    uint32_t imu_wb_read_count = 0;
    hw_baro_set_calibration(0.0f);
    hw_imu_set_calibration(vec3(0.0f), vec3(0.0f), vec3(1.0f));
    while (seconds_us() < calibration_end) {
        if (hw_baro_ready()) {
            BaroData data_baro = hw_baro_read();
            data_baro_accumulator.altitude += data_baro.altitude;
            baro_read_count++;

            IMUDataFIFO data_imu = hw_imu_integrate(state.q);
            data_imu_accumulator.wb = data_imu_accumulator.wb + (data_imu.wb * (float)data_imu.wb_count);
            imu_wb_read_count += data_imu.wb_count;
        }
    }
    hw_baro_set_calibration(data_baro_accumulator.altitude / (float)baro_read_count);
    hw_imu_set_calibration(data_imu_accumulator.wb / (float)imu_wb_read_count, vec3(0.0), vec3(1.0));
    hw_wifi_tx_log("Calibration complete.");

    /* -------------------------------- Main Loop ------------------------------- */
    bool loop_exit = false;
    while (!loop_exit) {
        // Guidance, navigation, and control from baro and IMU.
        if (hw_baro_ready()) {
            // Set state timers.
            state.time_delta = seconds_us() - state.time_current;
            state.time_current = seconds_us();

            // Data read resets timers.
            BaroData data_baro = hw_baro_read();
            state.baro_rate = (state.baro_rate - state.baro_curr) / driver_baro_config.rate_seconds;
            state.baro_curr = data_baro.altitude;

            IMUDataFIFO data_imu = hw_imu_integrate(state.q);
            state.ab = data_imu.ab;
            state.wb = data_imu.wb;
            state.dcmbe = data_imu.dcmbe;
            state.dcmeb = data_imu.dcmeb;
            state.q = data_imu.q1;
            state.eul_rate = (data_imu.eul - state.eul_curr) / driver_baro_config.rate_seconds;
            state.eul_curr = data_imu.eul;

            // Complementary filter estimate.
            // Note, assumes ab = ae.
            state.cpl_velocity = state.cpl_velocity_factor*(state.cpl_velocity + state.ab.x * driver_baro_config.rate_seconds) + (1.0f - state.cpl_velocity_factor)*state.baro_rate;
            state.cpl_altitude = state.cpl_altitude_factor*(state.cpl_altitude + state.cpl_velocity * driver_baro_config.rate_seconds) + (1.0f - state.cpl_altitude_factor)*state.baro_curr;

            // Kalman filter estimate.

            // Controls (from complementary estimate)
            // TODO.

            // Full state logged to PSRAM at GNC intervals when we're interested in the data. (Burn and coast phase of armed state)
            if (!state.flag_debug && state.flag_launch && !state.flag_apogee) hw_psram_append((uint8_t*)(&state), sizeof(ProgramStateFV));
        }

        // GPS data read and immediate attempt at LoRa transmission.
        if (hw_gps_ready()) {
            // Data read and pack.
            GPSData data_gps = hw_gps_read();
            state.latitude = data_gps.latitude;
            state.longitude = data_gps.longitude;
            state.siv = data_gps.siv;

            LoRaPacketUnpacked packet;
            packet.latitude = state.latitude;
            packet.longitude = state.longitude;
            packet.siv = state.siv;
            packet.altitude = state.baro_curr;
            packet.battery_voltage = hw_bat_volt();
            packet.flag_debug = state.flag_debug;
            packet.flag_launch = state.flag_launch;
            packet.flag_burnout = state.flag_burnout;
            packet.flag_apogee = state.flag_apogee;
            packet.cont1 = hw_gpio_pyro1_cont();
            packet.cont2 = hw_gpio_pyro2_cont();
            packet.cont3 = hw_gpio_pyro3_cont();
            packet.cont4 = hw_gpio_pyro4_cont();

            // Attempt transmit of packed data.
            // Does nothing if radio busy.
            hw_lora_transmit_buffer(pack_lora(packet).buffer, 8);
        }

        // WiFi transmission at fixed intervals when in debug mode.
        if (state.flag_debug && (seconds_us() > wifi_tx_next)) {
            wifi_tx_next = seconds_us() + wifi_tx_rate;
            hw_wifi_tx_data((uint8_t*)(&state), sizeof(ProgramStateFV));
        }

        // LoRa flag handling.
        // Flags aren't processed if armed and launch has happened to avoid in-flight disarm.
        // BREAKLOOP exits main loop immediately. If in debug, this leads to arming. If in armed, this disarms.
        // TODO HITL_ENABLE
        // TODO HITL_DISABLE
        if (hw_lora_transmit_pending()) { hw_lora_transmit_finish(); }
        if (hw_lora_receive_pending()) {
            uint8_t buffer[1];
            uint8_t bytes_read = hw_lora_receive_finish(buffer, 1);
            if (bytes_read && !(state.flag_launch && state.flag_debug)) { switch (buffer[1]) {
                case CMD_LORA_BREAKLOOP: { loop_exit = true; break; }
                case CMD_LORA_HITL_ENABLE: { break; }
                case CMD_LORA_HITL_DISABLE: { break; }
            }}
        }

        // Event handling while in armed mode.
        if (!state.flag_debug) {
            // Launch detection based off X axis G's.
            // This sets timers that work relative to launch time.
            if (!state.flag_launch && state.ab.x > threshold_launch_detect_gs * g) {
                state.flag_launch = true;
                timer_launch = seconds_us();
            }
            // Burnout detection runs after launch on time relative to launch.
            if (!state.flag_burnout && state.flag_launch && seconds_us() > timer_launch + threshold_burnout_time) {
                state.flag_burnout = true;
            }
            // Apogee detection has two possible methods.
            // One is a simple timer brom burnout.
            // The other is a smaller timer, after which velocity is monitored.
            // This functions as a safety net against erroneous late or early tiggering.
            //
            // Apogee detection also triggers drogue pyro (PYRO1) firing
            // and PSRAM save to flash.
            if (
                (!state.flag_apogee && state.flag_launch && seconds_us() > timer_launch + threshold_max_apogee_time) ||
                (!state.flag_apogee && state.flag_launch && seconds_us() > timer_launch + threshold_min_apogee_time && state.ve.x < 0.0f)
            ) {
                state.flag_apogee = true;
                hw_psram_save();
                hw_gpio_pyro1_fire(threshold_pyro_fire_time);
            }
            // Main charge (PYRO2) fires during descent after apogee.
            if (state.flag_apogee && state.xe.x < threshold_main_deploy) {
                hw_gpio_pyro2_fire(threshold_pyro_fire_time);
            }
        }

        // Disable pyros when their timers expire.
        hw_gpio_pyro_check_timers();
    }    
}