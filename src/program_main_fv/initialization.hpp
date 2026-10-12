#pragma once
#include <driver_main/_driver.hpp>
#include <program_main_fv/state.hpp>

/* -------------------------------------------------------------------------- */
/*                               Initialization                               */
/* -------------------------------------------------------------------------- */
void program_fv_initialization(ProgramStateFVMain& state) {
    // Initialize basic pins. (Init before OTA allows for startup blinks.)
    hw_gpio_init();
    hw_gpio_blink(10, 50);

    // Intialize wifi and do initial setup loop.
    // Loop is exited with a "mode setting" command over WiFi.
    // Mode 0: Exit loop.
    // Mode 1: Exit loop, format PSRAM data in flash.
    // Mode 2: Exit loop, read PSRAM flash data over wifi.
    // Mode 3: Exit loop, read PSRAM flash data over wifi, format PSRAM data in flash.
    uint32_t setup_mode;
    hw_wifi_on_ap("CYCLONE_V1_FV", "cyclone", "cyclone_v1_fv");
    while (true) {
        hw_gpio_blink(1, 50);
        hw_wifi_tx_log("Awaiting OTA.");
        hw_wifi_handle_ota();
        uint8_t byte = hw_wifi_rx_byte();
        if (byte == CMD_WIFI_OTA_CLEAR_MODE0) { hw_wifi_tx_log("Setup mode set to 0."); setup_mode = 0; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE1) { hw_wifi_tx_log("Setup mode set to 1."); setup_mode = 1; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE2) { hw_wifi_tx_log("Setup mode set to 2."); setup_mode = 2; break; }
        if (byte == CMD_WIFI_OTA_CLEAR_MODE3) { hw_wifi_tx_log("Setup mode set to 3."); setup_mode = 3; break; }
        delay(1000);
    }

    // Initialize all driver modules.
    // TODO Camera initialization?
    hw_baro_init();         hw_wifi_tx_log("Barometer initialized.");
    hw_gps_init();          hw_wifi_tx_log("GPS initialized.");
    hw_imu_init();          hw_wifi_tx_log("IMU initialized.");
    hw_lora_init();         hw_wifi_tx_log("LORA initialized.");
    hw_psram_init();        hw_wifi_tx_log("PSRAM initialized.");

    // Handle setup mode.
    bool setup_mode_read_data = (setup_mode == 2) || (setup_mode == 3);
    bool setup_mode_format_data = (setup_mode == 1) || (setup_mode = 3);
    if (setup_mode_read_data) {
        // Load data.
        hw_wifi_tx_log("Reading data.");
        uint8_t bytes_read = hw_psram_load();

        // TX read data over wifi.
        const float wifi_tx_rate = 1.0f / 25.0f;
        float wifi_tx_next = 0.0f;
        uint32_t psram_idx = 0;
        while (psram_idx < bytes_read) {
            if (seconds_us() > wifi_tx_next) {
                wifi_tx_next = seconds_us() + wifi_tx_rate;
                ProgramStateFVMain packet;
                hw_psram_copy((uint8_t*)(&packet), sizeof(ProgramStateFVMain), psram_idx);
                psram_idx += sizeof(ProgramStateFVMain);
                hw_wifi_tx_data((uint8_t*)(&psram_idx), sizeof(ProgramStateFVMain));
            }
        }

        // Clear PSRAM for later writes.
        hw_psram_clear();
    }
    if (setup_mode_format_data) {
        hw_wifi_tx_log("Formatting data.");
        hw_psram_format();
    }
}