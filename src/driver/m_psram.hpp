/* -------------------------------------------------------------------------- */
/*                              driver_psram.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for PSRAM on the main board, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <driver/m_base.hpp>

/* ---------------------------- Config and State ---------------------------- */
struct PSRAMConfig {
    const uint32_t psr_buffer_size = 1048576;

    uint8_t* psr_buffer;
    uint32_t psr_buffer_ptr;
} hwm_config_psram;

/* -------------------------------- Interface ------------------------------- */
// Allocates a PSRAM buffer and assigns to internal pointer.
void hwm_psram_init() {
    hwm_config_psram.psr_buffer = (uint8_t*)ps_malloc(hwm_config_psram.psr_buffer_size);
}

// Append contents of given buffer to PSRAM.
// Does nothing if data does not fit.
void hwm_psram_append(uint8_t* buff, uint32_t size) {
    if ((hwm_config_psram.psr_buffer_ptr + size) > hwm_config_psram.psr_buffer_size) return;
    memcpy(&hwm_config_psram.psr_buffer[hwm_config_psram.psr_buffer_ptr], &buff, size);
    hwm_config_psram.psr_buffer_ptr += size;
}

// Reads data of given size from PSRAM into buffer.
void hwm_psram_copy(uint8_t* buff, uint32_t size, uint32_t ptr) {
    memcpy(&buff, &hwm_config_psram.psr_buffer[ptr], size);
}

// Clear contents of PSRAM.
void hwm_psram_clear() { 
    hwm_config_psram.psr_buffer_ptr = 0;
}

// Clears PSRAM data from flash.
void hwm_psram_format() {
    LittleFS.begin(true);
    LittleFS.format();
    LittleFS.end();
}

// Populates ths PSRAM buffer from SPIFFS (LittleFS), returns bytes read (may be 0)
uint32_t hwm_psram_load() {
    LittleFS.begin(true);
    File file = LittleFS.open("/psr.bin", FILE_READ);
    uint8_t bytes_read = file.readBytes((char*)hwm_config_psram.psr_buffer, hwm_config_psram.psr_buffer_size); 
    file.close();
    LittleFS.end();
    return bytes_read;
}

// Writes current PSRAM buffer to SPIFFS (LittleFS), returns nothing.
void hwm_psram_save() {
    LittleFS.begin(true);
    File file = LittleFS.open("/psr.bin", FILE_WRITE);
    file.write(hwm_config_psram.psr_buffer, hwm_config_psram.psr_buffer_ptr);
    file.close();
    LittleFS.end();
}