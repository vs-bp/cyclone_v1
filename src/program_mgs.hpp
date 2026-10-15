#pragma once
#include <core/_core.hpp>
#include <driver/_driver.hpp>
#include <program_mfv.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateGSMain {
    uint32_t flag_display_mode = CMD_DISPLAY_MODE_NONE;
    uint32_t rx_wifi_count = 0;
    uint32_t rx_lora_count = 0;
    ProgramStateFVMain rx_wifi;
    LoRaPacketUnpacked rx_lora;
};

/* -------------------------------------------------------------------------- */
/*                                 Subroutines                                */
/* -------------------------------------------------------------------------- */
void __udp_handler(ProgramStateGSMain& state) {
    // Early exit if no packet or bytes RX.
    const uint32_t max_packet_size = 1024;
    uint8_t udp_buffer[max_packet_size];
    uint32_t udp_buffer_bytes = hwm_wifi_rx_buffer(udp_buffer, max_packet_size);
    if(!udp_buffer_bytes) return;

    // Log command case.
    if (udp_buffer[0] == CMD_WIFI_LOG) {
        // Print all remaining bytes after first.
        String packet_text = "";
        for (uint32_t i = 1; i < udp_buffer_bytes; i++) { packet_text += (char)udp_buffer[i]; }
        hwm_cam_serial_print(String("[FV] ") + packet_text + "\n");
    }
    // Data command case.
    if (udp_buffer[0] == CMD_WIFI_DATA) {
        if ((udp_buffer_bytes == sizeof(ProgramStateFVMain) + 1)) {
            memcpy(&state.rx_wifi, &udp_buffer[1], sizeof(ProgramStateFVMain));
            state.rx_wifi_count++;
        } else hwm_cam_serial_print("[GS] Invalid UDP Data Packet RX! (" + String(udp_buffer_bytes) + ") bytes\n");
    }
}

/* -------------------------------------------------------------------------- */
/*                                 Main Steps                                 */
/* -------------------------------------------------------------------------- */
void program_mgs_initialization(ProgramStateGSMain& state) {
    // Initial delay to allow CAM to stabilize and then
    // transmit to put into GS mode.
    hwm_gpio_init();
    delay(5000);
    
    // Intialize wifi and do initial setup loop.
    // Loop is exited with a "mode setting" command over relayed serial from CAM board.
    hwm_wifi_on_ap("CYCLONE_V1_GS", "cyclonepcb", "cyclone_v1_gs");
    while (true) {
        hwm_gpio_blink(1, 50);
        hwm_cam_serial_print("[GS] Awaiting OTA.\n");
        hwm_wifi_handle_ota();
        if (hwm_cam_serial_query() == CMD_SERIAL_OTA_CLEAR) break;
        delay(1000);
    }
    hwm_cam_serial_clear();
    hwm_wifi_tx_log("[GS] OTA cleared.\n");

    // Initialize relevant driver modules.
    hwm_lora_init();

    // Switch WiFi to station mode.
    hwm_wifi_off();
    delay(1000);
    hwm_wifi_on_client("CYCLONE_V1_FV", "cyclonepcb", "cyclone_v1_gs");
    while (!hwm_wifi_connected()) {
        hwm_cam_serial_print("[GS] Awaiting WiFi connection to FV.\n");
        delay(1000);
    }
    hwm_cam_serial_print("[GS] WiFi connected.\n");
}
void program_mgs_loop(ProgramStateGSMain state) {
    while (true) {
        // Handle LoRa flags. 
        // Transmit sets back into receive, receive saves data read to state.
        if (hwm_lora_transmit_pending()) { hwm_lora_transmit_finish(); }
        if (hwm_lora_receive_pending()) {      
            LoRaPacketPacked packet;
            uint8_t bytes = hwm_lora_receive_finish(packet.buffer, 8);
            if (bytes != 8) hwm_cam_serial_print("[GS] Invalid LoRa packet RX.\n");
            else {
                state.rx_lora = unpack_lora(packet);
                state.rx_lora_count++;
            }
        }
        
        // Handle serial commands.
        // Each incoming serial byte is interpreted as a command.
        // Each command reception, the command is cleared from the CAM board after handling and 
        // some text is printed to make it known the command was received.
        uint8_t cmd = hwm_cam_serial_query();
        switch (cmd) {
            case CMD_GENERIC_NONE            : { break; }
            case CMD_SERIAL_RESTART          : { hwm_cam_serial_print("[MAIN] Restarting ground station. \n"); ESP.restart(); break; }
            case CMD_LORA_BREAKLOOP          : { hwm_lora_transmit_byte(CMD_LORA_BREAKLOOP); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX BREAKLOOP command. \n"); break; }
            case CMD_LORA_HITL_ENABLE        : { hwm_lora_transmit_byte(CMD_LORA_HITL_ENABLE); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX HITL enable. \n"); break; }
            case CMD_LORA_HITL_DISABLE       : { hwm_lora_transmit_byte(CMD_LORA_HITL_DISABLE); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX HITL disable. \n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE0    : { hwm_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE0); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX setup mode 0 (Nread, Nfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE1    : { hwm_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE1); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX setup mode 1 (Nread, Yfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE2    : { hwm_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE2); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX setup mode 2 (Yread, Nfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE3    : { hwm_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE3); hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] TX setup mode 3 (Yread, Yfmt).\n"); break; }
            case CMD_DISPLAY_MODE_NONE       : { state.flag_display_mode = CMD_DISPLAY_MODE_NONE; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to None."); break; }
            case CMD_DISPLAY_MODE_GPS        : { state.flag_display_mode = CMD_DISPLAY_MODE_GPS; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to GPS."); break; }
            case CMD_DISPLAY_MODE_EULER      : { state.flag_display_mode = CMD_DISPLAY_MODE_EULER; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Euler."); break; }
            case CMD_DISPLAY_MODE_ACCEL      : { state.flag_display_mode = CMD_DISPLAY_MODE_ACCEL; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Accelerometers."); break; }
            case CMD_DISPLAY_MODE_GYRO       : { state.flag_display_mode = CMD_DISPLAY_MODE_GYRO; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Gyroscopes."); break; }
            case CMD_DISPLAY_MODE_CONTROLS   : { state.flag_display_mode = CMD_DISPLAY_MODE_CONTROLS; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Controls."); break;}
            case CMD_DISPLAY_MODE_BARO       : { state.flag_display_mode = CMD_DISPLAY_MODE_BARO; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Barometric."); break;}
            case CMD_DISPLAY_MODE_KALMAN     : { state.flag_display_mode = CMD_DISPLAY_MODE_KALMAN; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Kalman."); break;}
            case CMD_DISPLAY_MODE_CPL        : { state.flag_display_mode = CMD_DISPLAY_MODE_CPL; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Complementary."); break;}
            case CMD_DISPLAY_MODE_NET        : { state.flag_display_mode = CMD_DISPLAY_MODE_NET; hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Display mode set to Network."); break;}
            default                          : { hwm_cam_serial_clear(); hwm_cam_serial_print("[MAIN] Unknown command read. \n"); }
        }

        
        // Check incoming UDP packets for log or data commands.
        // Log commands get printed, and data command results get stored for use in the display state handler.
        // Early exit if no packet.
        //
        // This is deffered to a function to allow early exit.
        __udp_handler(state);

        // Handle display.
        switch (state.flag_display_mode) {
            // TODO Battery voltage.
            // TOOD UDP Stats.
            case CMD_DISPLAY_MODE_NONE       : { break; }
            case CMD_DISPLAY_MODE_GPS        : { hwm_cam_serial_print("[MAIN] GPS LATLONG SIV ALT " + String(state.rx_lora.latitude) + " " + String(state.rx_lora.longitude) + " " + String(state.rx_lora.siv) + " " + String(state.rx_lora.altitude) + "\n"); break; }
            case CMD_DISPLAY_MODE_BARO       : { hwm_cam_serial_print("[MAIN] BARO CURR RATE " + String(state.rx_wifi.baro_curr) + " " + String(state.rx_wifi.baro_rate) + "\n"); break; }
            case CMD_DISPLAY_MODE_EULER      : { hwm_cam_serial_print("[MAIN] EUL RPY " + String(state.rx_wifi.eul_curr.x) + " " + String(state.rx_wifi.eul_curr.y) + " " + String(state.rx_wifi.eul_curr.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_ACCEL      : { hwm_cam_serial_print("[MAIN] ACCEL XYZ " + String(state.rx_wifi.ab.x) + " " + String(state.rx_wifi.ab.y) + " " + String(state.rx_wifi.ab.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_GYRO       : { hwm_cam_serial_print("[MAIN] GYRO XYZ " + String(state.rx_wifi.wb.x) + " " + String(state.rx_wifi.wb.x) + " " + String(state.rx_wifi.wb.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_CONTROLS   : { hwm_cam_serial_print("[MAIN] CTRL " + String(state.rx_wifi.canard_angle) + "\n"); break; }
            case CMD_DISPLAY_MODE_KALMAN     : { hwm_cam_serial_print("[MAIN] KAMAN XVA " + String(state.rx_wifi.kalman_x1.x) + " " + String(state.rx_wifi.kalman_x1.y) + " " + String(state.rx_wifi.kalman_x1.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_CPL        : { hwm_cam_serial_print("[MAIN] CPL XV " + String(state.rx_wifi.cpl_altitude) + " " + String(state.rx_wifi.cpl_velocity) + "\n"); break; }
            case CMD_DISPLAY_MODE_NET        : { hwm_cam_serial_print("[MAIN] NET WIFI LORA " + String(state.rx_wifi_count) + " " + String(state.rx_lora_count) + "\n"); break; }
            default: { break; }
        }

        // Rate limiting.
        delay(100);
    }
}

/* -------------------------------------------------------------------------- */
/*                                Main Function                               */
/* -------------------------------------------------------------------------- */
void program_mgs() {
    ProgramStateGSMain state_gs;
    program_mgs_initialization(state_gs);
    program_mgs_loop(state_gs);
}