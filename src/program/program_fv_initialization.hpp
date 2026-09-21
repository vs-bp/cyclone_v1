#pragma once
#include <driver/driver.hpp>
#include <program/program_fv_state.hpp>

/* -------------------------------------------------------------------------- */
/*                               Initialization                               */
/* -------------------------------------------------------------------------- */
void program_fv_initialization(ProgramStateFV& state) {
    // Intialize wifi and do initial setup loop.
    // Loop is exited with a "mode setting" command over WiFi.
    // Mode 0: Exit loop.
    // Mode 1: Exit loop, format PSRAM data in flash.
    // Mode 2: Exit loop, read PSRAM flash data over wifi.
    // Mode 3: Exit loop, read PSRAM flash data over wifi, format PSRAM data in flash.
    uint32_t setup_mode;
    hw_wifi_on_ap("CYCLONE_V1_FV", "cyclone");
    hw_wifi_begin_ota("cyclone_v1_fv");
    while (true) {
        hw_gpio_blink(1, 50);
        hw_wifi_tx_string("Awaiting OTA.");
        hw_wifi_handle_ota();
        if (hw_wifi_rx_byte() == CMD_WIFI_OTA_CLEAR_MODE0) { hw_wifi_tx_string("Setup mode set to 0."); setup_mode = 0; break; }
        if (hw_wifi_rx_byte() == CMD_WIFI_OTA_CLEAR_MODE1) { hw_wifi_tx_string("Setup mode set to 1."); setup_mode = 1; break; }
        if (hw_wifi_rx_byte() == CMD_WIFI_OTA_CLEAR_MODE2) { hw_wifi_tx_string("Setup mode set to 2."); setup_mode = 2; break; }
        if (hw_wifi_rx_byte() == CMD_WIFI_OTA_CLEAR_MODE3) { hw_wifi_tx_string("Setup mode set to 3."); setup_mode = 3; break; }
        delay(1000);
    }

    // Initialize all driver modules.
    hw_baro_init();
    hw_cam_init();
    hw_gpio_init();
    hw_gps_init();
    hw_imu_init();
    hw_lora_init();
    hw_psram_init();

    // Handle setup mode.
    bool setup_mode_read_data = (setup_mode == 2) || (setup_mode == 3);
    bool setup_mode_format_data = (setup_mode == 1) || (setup_mode = 3);
    if (setup_mode_read_data) {
        hw_wifi_tx_string("Reading data.");
        hw_psram_load();
        // TODO read loaded PSRAM data back through wifi.
    }
    if (setup_mode_format_data) {
        hw_wifi_tx_string("Formatting data.");
        hw_psram_format();
    }
}