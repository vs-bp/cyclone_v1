#pragma once
#include <core/_core.hpp>
#include <driver/_driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                 Main Steps                                 */
/* -------------------------------------------------------------------------- */
void program_cgs_initialization() {
    Serial.begin(115200);
    hwc_i2c_init();
}
void program_cgs_loop() {
    while (true) {
        // If serial data in is available, we read it in.
        // Data in is always assumed to be a single-byte.
        //
        // This single byte is stored as the serial cmd flag,
        // and is outputted whenever a byte is requested from the device (and cleared by I2C below)
        if (Serial.available()) {
            // Read in byte.
            uint8_t cmd = Serial.read();
            hwc_i2c_set_serial_cmd_flag(cmd);
            Serial.print("[CAM] Serial CMD: " + String(cmd, HEX) + "\n");
            // Discard remaining bytes.
            while (Serial.available()) { Serial.read(); }
        }

        // I2C Buffer is populated by handler interrupts.
        // First byte represents command and following are contextual.
        if (hwc_i2c_buffer_len() != 0) {
            uint8_t cmd = hwc_i2c_buffer()[0];
            switch (cmd) {
                // Print all following bytes to serial.
                case CMD_CAM_I2C_PRINT: { 
                    for (uint32_t i = 1; i < hwc_i2c_buffer_len(); i++) { Serial.print((char)hwc_i2c_buffer()[i]); } 
                    break; 
                }
                // Clears any commands which have been held by serial (acknowledging their execution)
                case CMD_CAM_I2C_CLEAR: {
                    hwc_i2c_set_serial_cmd_flag(CMD_GENERIC_NONE);
                    Serial.print("[CAM] Serial CMD cleared.\n"); 
                    break;
                }
            }
            // Flush buffer after use.
            hwc_i2c_buffer_flush();
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                                Main Function                               */
/* -------------------------------------------------------------------------- */
void program_cgs() {
    program_cgs_initialization();
    program_cgs_loop();
}