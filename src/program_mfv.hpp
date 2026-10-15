#pragma once
#include <core/_core.hpp>
#include <driver/_driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateFVMain {
    // Timing.
    float time_current = 0.0f;              // seconds.
    float time_delta = 0.0f;                // seconds.
    // Event flags.
    bool flag_debug = false;
    bool flag_launch = false;
    bool flag_burnout = false;
    bool flag_apogee = false;
    
    // Fundamental state.
    vec3 ab = vec3(0.0f, 0.0f, 0.0f);       // meters / (second ^ 2).
    vec3 wb = vec3(0.0f, 0.0f, 0.0f);       // radians / second.
    vec3 xe = vec3(0.0f, 0.0f, 0.0f);       // meters.
    vec3 ve = vec3(0.0f, 0.0f, 0.0f);       // meters / second.
    vec3 ae = vec3(0.0f, 0.0f, 0.0f);       // meters / (second ^ 2).
    vec4 q = vec4(1.0f, 0.0f, 0.0f, 0.0f);  // unitless.
    vec3 eul_curr = vec3(0.0f, 0.0f, 0.0f); // radians.
    vec3 eul_rate = vec3(0.0f, 0.0f, 0.0f); // radians.
    mat3 dcmbe = mat3(1.0f);                // unitless.
    mat3 dcmeb = mat3(1.0f);                // unitless.

    // Altitude variables.
    float baro_curr = 0.0f;                 // meters.
    float baro_rate = 0.0f;                 // meters / second.

    // Complementary filter estimates.
    const float cpl_altitude_factor = 0.95f; // unitless.
    const float cpl_velocity_factor = 0.05f; // unitless.
    const float cpl_ab_threshold = 1.0f * g; // meters / (second ^ 2)
    float cpl_altitude = 0.0f;               // meters.
    float cpl_velocity = 0.0f;               // meters / second.

    // Kalman filter estimate.
    // State vector: (Xe, Ve, Ae) all on vertical axis in earth space.
    const mat3 kalman_q = mat3(vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, 0.0, 1.0));
    const mat3 kalman_r = mat3(vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0), vec3(0.0, 0.0, 1.0));
    vec3 kalman_z1 = vec3(0.0);
    vec3 kalman_x1 = vec3(0.0);
    mat3 kalman_p1 = mat3(1.0);

    // GPS variables.
    float latitude = 0.0f;                   // degrees.
    float longitude = 0.0f;                  // degrees.
    uint8_t siv = 0;                         // unitless.

    // Control deflections.
    float canard_angle = 0.0f;               // degrees.
};

/* -------------------------------------------------------------------------- */
/*                                 Main Steps                                 */
/* -------------------------------------------------------------------------- */
void program_mfv_initialization() {
    // Initialize basic pins. (Init before OTA allows for startup blinks.)
    hwm_gpio_init();
    hwm_gpio_blink(10, 50);

    // Intialize wifi and do initial setup loop.
    // Loop is exited with a "mode setting" command over WiFi.
    // Mode 0: Exit loop.
    // Mode 1: Exit loop, format PSRAM data in flash.
    // Mode 2: Exit loop, read PSRAM flash data over wifi.
    // Mode 3: Exit loop, read PSRAM flash data over wifi, format PSRAM data in flash.
    uint32_t setup_mode;
    hwm_wifi_on_ap("CYCLONE_V1_FV", "cyclonepcb", "cyclone_v1_fv");
    while (true) {
        hwm_gpio_blink(1, 50);
        hwm_wifi_tx_log("Awaiting OTA.");
        hwm_wifi_handle_ota();
        uint8_t byte = hwm_wifi_rx_byte();
        if (byte == CMD_WIFI_OTA_CLEAR_MODE0) { hwm_wifi_tx_log("Setup mode set to 0."); setup_mode = 0; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE1) { hwm_wifi_tx_log("Setup mode set to 1."); setup_mode = 1; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE2) { hwm_wifi_tx_log("Setup mode set to 2."); setup_mode = 2; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE3) { hwm_wifi_tx_log("Setup mode set to 3."); setup_mode = 3; break; }
        delay(1000);
    }

    // Initialize all driver modules.
    // TODO Camera initialization?
    hwm_baro_init();         hwm_wifi_tx_log("Barometer initialized.");
    hwm_gps_init();          hwm_wifi_tx_log("GPS initialized.");
    hwm_imu_init();          hwm_wifi_tx_log("IMU initialized.");
    hwm_lora_init();         hwm_wifi_tx_log("LORA initialized.");
    hwm_psram_init();        hwm_wifi_tx_log("PSRAM initialized.");

    // Handle setup mode.
    bool setup_mode_read_data = (setup_mode == 2) || (setup_mode == 3);
    bool setup_mode_format_data = (setup_mode == 1) || (setup_mode = 3);
    if (setup_mode_read_data) {
        // Load data.
        hwm_wifi_tx_log("Reading data.");
        uint8_t bytes_read = hwm_psram_load();

        // TX read data over wifi.
        const float wifi_tx_rate = 1.0f / 25.0f;
        float wifi_tx_next = 0.0f;
        uint32_t psram_idx = 0;
        while (psram_idx < bytes_read) {
            if (seconds_us() > wifi_tx_next) {
                wifi_tx_next = seconds_us() + wifi_tx_rate;
                ProgramStateFVMain packet;
                hwm_psram_copy((uint8_t*)(&packet), sizeof(ProgramStateFVMain), psram_idx);
                psram_idx += sizeof(ProgramStateFVMain);
                hwm_wifi_tx_data((uint8_t*)(&psram_idx), sizeof(ProgramStateFVMain));
            }
        }

        // Clear PSRAM for later writes.
        hwm_psram_clear();
    }
    if (setup_mode_format_data) {
        hwm_wifi_tx_log("Formatting data.");
        hwm_psram_format();
    }
}
void program_mfv_loop(ProgramStateFVMain& state, bool debug) {
    state.flag_debug = debug;

    /* ----------------------------- Initialization ----------------------------- */
    // Conditional disabling of wifi if intending to arm for flight.
    // Wifi logging of current loop entered.
    if (state.flag_debug) {
        hwm_wifi_tx_log("Debug loop entered.");
    } else {
        hwm_wifi_tx_log("Armed loop entered.");
        hwm_wifi_off();
    }

    /* ---------------------------- Flags, Thresholds, and Timers ---------------------------- */
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
    hwm_wifi_tx_log("Starting calibration.");
    const float calibration_time = 30.0f;
    float calibration_end = seconds_us() + calibration_time;
    BaroData data_baro_accumulator;
    IMUDataFIFO data_imu_accumulator;
    uint32_t baro_read_count = 0;
    uint32_t imu_wb_read_count = 0;
    hwm_baro_set_calibration(0.0f);
    hwm_imu_set_calibration(vec3(0.0f), vec3(0.0f), vec3(1.0f));
    while (seconds_us() < calibration_end) {
        if (hwm_baro_ready()) {
            BaroData data_baro = hwm_baro_read();
            data_baro_accumulator.altitude += data_baro.altitude;
            baro_read_count++;

            IMUDataFIFO data_imu = hwm_imu_integrate(state.q);
            data_imu_accumulator.wb = data_imu_accumulator.wb + (data_imu.wb * (float)data_imu.wb_count);
            imu_wb_read_count += data_imu.wb_count;
        }
    }
    hwm_baro_set_calibration(data_baro_accumulator.altitude / (float)baro_read_count);
    hwm_imu_set_calibration(data_imu_accumulator.wb / (float)imu_wb_read_count, vec3(0.0), vec3(1.0));
    hwm_wifi_tx_log("Calibration complete.");

    /* -------------------------------- Main Loop ------------------------------- */
    bool loop_exit = false;
    while (!loop_exit) {
        // Guidance, navigation, and control from baro and IMU.
        if (hwm_baro_ready()) {
            // Set state timers.
            state.time_delta = seconds_us() - state.time_current;
            state.time_current = seconds_us();

            // Data read resets timers.
            BaroData data_baro = hwm_baro_read();
            state.baro_rate = (data_baro.altitude - state.baro_curr) / hwm_config_baro.rate_seconds;
            state.baro_curr = data_baro.altitude;

            // Data read from IMU.
            IMUDataFIFO data_imu = hwm_imu_integrate(state.q);
            state.ab = data_imu.ab;
            state.wb = data_imu.wb;
            state.dcmbe = data_imu.dcmbe;
            state.dcmeb = data_imu.dcmeb;
            state.q = data_imu.q1;
            state.eul_rate = (data_imu.eul - state.eul_curr) / hwm_config_baro.rate_seconds;
            state.eul_curr = data_imu.eul;

            // Earth-axes acceleration from rotation.
            state.ae = state.dcmeb * state.ab;

            // Integrate earth-axes values from acceleration (dead reckoning)
            state.ve = state.ve + state.ae * hwm_config_baro.rate_seconds;
            state.xe = state.xe + state.ve * hwm_config_baro.rate_seconds;

            // Complementary filter estimate.
            // Note, assumes ab = ae.
            state.cpl_velocity = state.cpl_velocity_factor*(state.cpl_velocity + state.ab.x * hwm_config_baro.rate_seconds) + (1.0f - state.cpl_velocity_factor)*state.baro_rate;
            state.cpl_altitude = state.cpl_altitude_factor*(state.cpl_altitude + state.cpl_velocity * hwm_config_baro.rate_seconds) + (1.0f - state.cpl_altitude_factor)*state.baro_curr;

            // Kalman filter estimate.
            float rate_kalman = hwm_config_baro.rate_seconds;
            mat3 A = mat3(
                vec3(1.0, rate_kalman, 0.0), 
                vec3(0.0, 1.0, rate_kalman), 
                vec3(0.0, 0.0, 1.0)
            );
            vec3 B = vec3(0.0);
            vec3 x1m = A*state.kalman_x1 + B;
            mat3 p1m = A*A*state.kalman_p1 + state.kalman_q;
            mat3 k1 = p1m * (state.kalman_p1 + state.kalman_r).inverse();
            vec3 z1 = vec3(state.baro_curr, state.baro_rate, state.ae.x);
            state.kalman_x1 = (mat3(1.0) - k1)*x1m + k1*z1;
            state.kalman_p1 = (mat3(1.0) - k1)*p1m;

            // Controls (from complementary estimate)
            // TODO.

            // Full state logged to PSRAM at GNC intervals when we're interested in the data. (Burn and coast phase of armed state)
            // Full state also logged over WiFi when in debug mode.
            if (!state.flag_debug && state.flag_launch && !state.flag_apogee) hwm_psram_append((uint8_t*)(&state), sizeof(ProgramStateFVMain));
            if (state.flag_debug) hwm_wifi_tx_data((uint8_t*)(&state), sizeof(ProgramStateFVMain));
        }

        // GPS data read and immediate attempt at LoRa transmission.
        if (hwm_gps_ready()) {
            // Data read and pack.
            GPSData data_gps = hwm_gps_read();
            state.latitude = data_gps.latitude;
            state.longitude = data_gps.longitude;
            state.siv = data_gps.siv;

            LoRaPacketUnpacked packet;
            packet.latitude = state.latitude;
            packet.longitude = state.longitude;
            packet.siv = state.siv;
            packet.altitude = state.baro_curr;
            packet.battery_voltage = hwm_bat_volt();
            packet.flag_debug = state.flag_debug;
            packet.flag_launch = state.flag_launch;
            packet.flag_burnout = state.flag_burnout;
            packet.flag_apogee = state.flag_apogee;
            packet.cont1 = hwm_gpio_pyro1_cont();
            packet.cont2 = hwm_gpio_pyro2_cont();
            packet.cont3 = hwm_gpio_pyro3_cont();
            packet.cont4 = hwm_gpio_pyro4_cont();

            // Attempt transmit of packed data.
            // Does nothing if radio busy.
            hwm_lora_transmit_buffer(pack_lora(packet).buffer, 8);
        }

        // LoRa flag handling.
        // Flags aren't processed if armed and launch has happened to avoid in-flight disarm.
        // BREAKLOOP exits main loop immediately. If in debug, this leads to arming. If in armed, this disarms.
        // TODO HITL_ENABLE
        // TODO HITL_DISABLE
        if (hwm_lora_transmit_pending()) { hwm_lora_transmit_finish(); }
        if (hwm_lora_receive_pending()) {
            uint8_t buffer[1];
            uint8_t bytes_read = hwm_lora_receive_finish(buffer, 1);
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
                hwm_psram_save();
                hwm_gpio_pyro1_fire(threshold_pyro_fire_time);
            }
            // Main charge (PYRO2) fires during descent after apogee.
            if (state.flag_apogee && state.xe.x < threshold_main_deploy) {
                hwm_gpio_pyro2_fire(threshold_pyro_fire_time);
            }
        }

        // Disable pyros when their timers expire.
        hwm_gpio_pyro_check_timers();
    }    
}
void program_mfv_disarmed() {
    while (true) {
        esp_sleep_enable_timer_wakeup(1000000ULL * 60ULL * 60ULL * 24ULL);
        esp_deep_sleep_start();
    }
}
/* -------------------------------------------------------------------------- */
/*                                Main Function                               */
/* -------------------------------------------------------------------------- */
void program_mfv() {
    ProgramStateFVMain state;
    program_mfv_initialization();
    program_mfv_loop(state, true);
    program_mfv_loop(state, false);
    program_mfv_disarmed();
}