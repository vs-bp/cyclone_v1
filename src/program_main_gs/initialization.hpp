#pragma once
#include <driver_main/_driver.hpp>
#include <program_main_gs/state.hpp>

/* -------------------------------------------------------------------------- */
/*                               Initialization                               */
/* -------------------------------------------------------------------------- */
void program_gs_initialization(ProgramStateGSMain& state) {
    // Initial delay to allow CAM to stabilize and then
    // transmit to put into GS mode.
    hw_gpio_init();
    delay(5000);
    
    // Intialize wifi and do initial setup loop.
    // Loop is exited with a "mode setting" command over relayed serial from CAM board.
    hw_wifi_on_ap("CYCLONE_V1_GS", "cyclonepcb", "cyclone_v1_gs");
    while (true) {
        hw_gpio_blink(1, 50);
        hw_cam_serial_print("[GS] Awaiting OTA.\n");
        hw_wifi_handle_ota();
        if (hw_cam_serial_query() == CMD_SERIAL_OTA_CLEAR) break;
        delay(1000);
    }
    hw_cam_serial_clear();
    hw_wifi_tx_log("[GS] OTA cleared.\n");

    // Initialize relevant driver modules.
    hw_lora_init();

    // Switch WiFi to station mode.
    hw_wifi_off();
    delay(1000);
    hw_wifi_on_client("CYCLONE_V1_FV", "cyclonepcb", "cyclone_v1_gs");
    while (!hw_wifi_connected()) {
        hw_cam_serial_print("[GS] Awaiting WiFi connection to FV.\n");
        delay(1000);
    }
    hw_cam_serial_print("[GS] WiFi connected.\n");
}