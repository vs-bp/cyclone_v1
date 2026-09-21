/* -------------------------------------------------------------------------- */
/*                              driver_psram.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for PSRAM on the main board, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* ---------------------------- Config and State ---------------------------- */
struct PSRAMConfig {
    const uint32_t psr_buffer_size = 1048576;

    uint8_t* psr_buffer;
    uint32_t psr_buffer_ptr;
} psram_config;

/* -------------------------------- Interface ------------------------------- */
// Allocates a PSRAM buffer and assigns to internal pointer.
void hw_psram_init() {
    psram_config.psr_buffer = (uint8_t*)ps_malloc(psram_config.psr_buffer_size);
}

// Append contents of given buffer to PSRAM.
void hw_psram_append(uint8_t* buff, uint32_t size) {
    memcpy(&psram_config.psr_buffer[psram_config.psr_buffer_ptr], &buff, size);
}

// Clear contents of PSRAM.
void hw_psram_clear() { 
    psram_config.psr_buffer_ptr = 0;
}

// Clears PSRAM data from flash.
void hw_psram_format() {
    LittleFS.begin(true);
    LittleFS.format();
    LittleFS.end();
}

// Populates ths PSRAM buffer from SPIFFS (LittleFS), returns bytes read (may be 0)
uint32_t hw_psram_load() {
    LittleFS.begin(true);
    File file = LittleFS.open("/psr.bin", FILE_READ);
    uint8_t bytes_read = file.readBytes((char*)psram_config.psr_buffer, psram_config.psr_buffer_size); 
    file.close();
    LittleFS.end();
    return bytes_read;
}

// Writes current PSRAM buffer to SPIFFS (LittleFS), returns nothing.
void hw_psram_save() {
    LittleFS.begin(true);
    File file = LittleFS.open("/psr.bin", FILE_WRITE);
    file.write(psram_config.psr_buffer, psram_config.psr_buffer_ptr);
    file.close();
    LittleFS.end();
}