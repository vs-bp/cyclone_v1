#pragma once
#include <driver/driver.hpp>
#include <program/program_gs_state.hpp>

/* -------------------------------------------------------------------------- */
/*                                 Subroutines                                */
/* -------------------------------------------------------------------------- */
void __udp_handler(ProgramStateGS& state) {
    // Early exit if no packet or bytes RX.
    uint8_t udp_buffer[256];
    uint8_t udp_buffer_bytes = hw_wifi_rx_buffer(udp_buffer, 256);
    if(!udp_buffer_bytes) return;

    // Log command case.
    if (udp_buffer[0] == CMD_WIFI_LOG) {
        // Print all remaining bytes after first.
        String packet_text = "";
        for (uint32_t i = 1; i < udp_buffer_bytes; i++) { packet_text += (char)udp_buffer[i]; }
        hw_cam_serial_print(String("[FV] ") + packet_text + "\n");
    }
    // Data command case.
    if ((udp_buffer[0] == CMD_WIFI_DATA) && (udp_buffer_bytes == sizeof(ProgramStateFV) + 1)) {
        memcpy(&state.rx_wifi, &udp_buffer, sizeof(ProgramStateFV));
    }
}

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */
void program_gs_loop(ProgramStateGS state) {
    while (true) {
        // Handle LoRa flags. 
        // Transmit sets back into receive, receive saves data read to state.
        if (hw_lora_transmit_pending()) { hw_lora_transmit_finish(); }
        if (hw_lora_receive_pending()) {      
            LoRaPacketPacked packet;
            uint8_t bytes = hw_lora_receive_finish(packet.buffer, 8);
            if (bytes != 8) hw_cam_serial_print("[GS] Invalid LoRa packet RX.\n");
            else state.rx_lora = unpack_lora(packet);
        }
        
        // Handle serial commands.
        // Each incoming serial byte is interpreted as a command.
        // Each command reception, the command is cleared from the CAM board after handling and 
        // some text is printed to make it known the command was received.
        uint8_t cmd = hw_cam_serial_query();
        switch (cmd) {
            case CMD_GENERIC_NONE            : { break; }
            case CMD_LORA_BREAKLOOP        : { hw_lora_transmit_byte(CMD_LORA_BREAKLOOP); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX BREAKLOOP command. \n"); break; }
            case CMD_LORA_HITL_ENABLE  : { hw_lora_transmit_byte(CMD_LORA_HITL_ENABLE); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX HITL enable. \n"); break; }
            case CMD_LORA_HITL_DISABLE : { hw_lora_transmit_byte(CMD_LORA_HITL_DISABLE); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX HITL disable. \n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE0 : { hw_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE0); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX setup mode 1 (Nread, Nfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE1 : { hw_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE1); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX setup mode 2 (Nread, Yfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE2 : { hw_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE2); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX setup mode 3 (Yread, Nfmt).\n"); break; }
            case CMD_WIFI_OTA_CLEAR_MODE3 : { hw_wifi_tx_cmd(CMD_WIFI_OTA_CLEAR_MODE3); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX setup mode 4 (Yread, Yfmt).\n"); break; }
            case CMD_DISPLAY_MODE_NONE       : { state.flag_display_mode = CMD_DISPLAY_MODE_NONE; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to None."); break; }
            case CMD_DISPLAY_MODE_GPS        : { state.flag_display_mode = CMD_DISPLAY_MODE_GPS; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to GPS."); break; }
            case CMD_DISPLAY_MODE_ALTITUDE   : { state.flag_display_mode = CMD_DISPLAY_MODE_ALTITUDE; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to Altitude."); break; }
            case CMD_DISPLAY_MODE_EULER      : { state.flag_display_mode = CMD_DISPLAY_MODE_EULER; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to Euler."); break; }
            case CMD_DISPLAY_MODE_ACCEL      : { state.flag_display_mode = CMD_DISPLAY_MODE_ACCEL; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to Accelerometers."); break; }
            case CMD_DISPLAY_MODE_GYRO       : { state.flag_display_mode = CMD_DISPLAY_MODE_GYRO; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to Gyroscopes."); break; }
            case CMD_DISPLAY_MODE_CONTROLS   : { state.flag_display_mode = CMD_DISPLAY_MODE_CONTROLS; hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Display mode set to Controls."); break;}
            default:                          { hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Unknown command read. \n"); }
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
            case CMD_DISPLAY_MODE_NONE       : { break; }
            case CMD_DISPLAY_MODE_GPS        : { hw_cam_serial_print("[MAIN] GPS LATLONG SIV " + String(state.rx_lora.latitude) + " " + String(state.rx_lora.longitude) + " " + String(state.rx_lora.siv) + "\n"); break; }
            case CMD_DISPLAY_MODE_ALTITUDE   : { hw_cam_serial_print("[MAIN] ALT " + String(state.rx_lora.altitude) + "\n"); break; }
            case CMD_DISPLAY_MODE_EULER      : { hw_cam_serial_print("[MAIN] EUL RPY " + String(state.rx_wifi.eul_curr.x) + " " + String(state.rx_wifi.eul_curr.y) + " " + String(state.rx_wifi.eul_curr.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_ACCEL      : { hw_cam_serial_print("[MAIN] ACCEL XYZ " + String(state.rx_wifi.ab.x) + " " + String(state.rx_wifi.ab.y) + " " + String(state.rx_wifi.ab.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_GYRO       : { hw_cam_serial_print("[MAIN] GYRO XYZ " + String(state.rx_wifi.wb.x) + " " + String(state.rx_wifi.wb.x) + " " + String(state.rx_wifi.wb.z) + "\n"); break; }
            case CMD_DISPLAY_MODE_CONTROLS   : { hw_cam_serial_print("[MAIN] CTRL " + String(state.rx_wifi.canard_angle) + "\n"); break; }
            default: { break; }
        }

        // Rate limiting.
        delay(100);
    }
}