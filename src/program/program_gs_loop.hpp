#pragma once
#include <driver/driver.hpp>
#include <program/program_gs_state.hpp>

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */
void program_gs_loop(ProgramStateGS state) {
    while (true) {
        // Handle LoRa flags.
        if (hw_lora_transmit_action_complete()) {
            // TODO.
        }
        if (hw_lora_receive_action_complete()) {
            // TODO.
        }
        
        // Handle serial commands.
        // Each incoming serial byte is interpreted as a command.
        // Each command reception, the command is cleared from the CAM board after handling and 
        // some text is printed to make it known the command was received.
        uint8_t cmd = hw_cam_serial_query();
        switch (cmd) {
            case CMD_LORA_DEBUG        : { hw_lora_transmit_byte(CMD_LORA_DEBUG); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX DISARMED state. \n"); break; }
            case CMD_LORA_ARMED        : { hw_lora_transmit_byte(CMD_LORA_ARMED); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX ARMED state. \n"); break; }
            case CMD_LORA_DISARMED     : { hw_lora_transmit_byte(CMD_LORA_DISARMED); hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] TX DEBUG state. \n"); break; }
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
            case CMD_GENERIC_NONE            : { break; }
            default:                          { hw_cam_serial_clear(); hw_cam_serial_print("[MAIN] Unknown command read. \n"); }
        }

        
        // Check incoming UDP packets for log or data commands.
        // Log commands get printed, and data command results get stored for use in the display state handler.
        // Early exit if no packet.
        /*
        uint32_t udp_packet_bytes = udp.parsePacket();
        if (!udp_packet_bytes) return;

        // Early exit if no bytes read into buffer.
        uint8_t udp_buffer[256];
        uint32_t udp_buffer_bytes = udp.read(udp_buffer, 256);
        if (udp_buffer_bytes <= 1) return;
        if (udp_buffer_bytes == 256) return hw_cam_serial_print("[MAIN] Oversize UDP packet received.\n");

        // Log command case.
        if (udp_buffer[0] == CMD_FV_TX_WIFI_LOG) {
            // Print all remaining bytes after first.
            String packet_text = "";
            for (uint32_t i = 1; i < udp_buffer_bytes; i++) { packet_text += (char)udp_buffer[i]; }
            hw_cam_serial_print(String("[VEHICLE] ") + packet_text + "\n");
        }
        if ((udp_buffer[0] == CMD_FV_TX_WIFI_DATA) && (udp_buffer_bytes == sizeof(CMD_TX_DATA_BUFFER) + 1)) {
            // Read in data buffer.
            CMD_TX_DATA_BUFFER buffer;
            memcpy(&buffer, &udp_buffer[1], sizeof(CMD_TX_DATA_BUFFER));

            // Only the data that is both not present in LORA and actually
            // displayed is taken from the buffer.
            vhc_eul_r = buffer.eul_r;
            vhc_eul_p = buffer.eul_p;
            vhc_eul_y = buffer.eul_y;
            vhc_ab_x = buffer.ab_x;
            vhc_ab_y = buffer.ab_y;
            vhc_ab_z = buffer.ab_z;
            vhc_wb_x = buffer.wb_x;
            vhc_wb_y = buffer.wb_y;
            vhc_wb_z = buffer.wb_z;
            vhc_canard_angle = buffer.angle;
        }
        */

        // Handle display.
        switch (state.flag_display_mode) {
            case CMD_DISPLAY_MODE_NONE       : { break; }
            case CMD_DISPLAY_MODE_GPS        : { hw_cam_serial_print("[MAIN] GPS LATLONG SIV " + String(state.rx_lora.latitude) + " " + String(state.rx_lora.longitude) + " " + String(state.rx_lora.siv) + "\n"); break; }
            case CMD_DISPLAY_MODE_ALTITUDE   : { hw_cam_serial_print("[MAIN] ALT " + String(state.rx_lora.baro_curr) + "\n"); break; }
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