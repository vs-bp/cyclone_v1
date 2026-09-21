/* -------------------------------------------------------------------------- */
/*                               driver_lora.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the lora module, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* ---------------------------- State and Config ---------------------------- */
struct LoRaConfig {
    const float frequency = 915.0f;
    const float bandwidth = 125.0f;
    const uint32_t power = 22;
    const uint32_t coding_rate = 22;
    const uint32_t spreading_factor = 7;
    const uint32_t preamble_length = 8;
    const float tcxo_voltage = 1.8f;
    const uint8_t sync_word = RADIOLIB_SX126X_SYNC_WORD_PRIVATE;
    const bool use_reg_ldo = false;
    const uint32_t buffer_size = 128;

    SPIClass spi_obj = SPIClass(HSPI);
    SX1262 lora_obj = new Module(PN_RF_NSS, PN_RF_DIO1, PN_RF_NRST, PN_RF_BUSY, spi_obj);

    bool action_complete = false;
    bool transmit_mode = false;
} driver_lora_config;

/* ------------------------ Interface Implementations ----------------------- */
// Init function defines DIO1 interrupt as just setting action complete flag to true.
// Radio starts in receive mode.
ICACHE_RAM_ATTR void int_dio1() { driver_lora_config.action_complete = true; }
void hw_lora_init() {
  pinMode(PN_SPI_SCK, OUTPUT);
  pinMode(PN_SPI_MISO, INPUT);
  pinMode(PN_SPI_MOSI, OUTPUT);
  pinMode(PN_RF_NSS, OUTPUT);
  driver_lora_config.spi_obj.begin(PN_SPI_SCK, PN_SPI_MISO, PN_SPI_MOSI, PN_RF_NSS);
  driver_lora_config.lora_obj.begin(
    driver_lora_config.frequency,
    driver_lora_config.bandwidth,
    driver_lora_config.spreading_factor,
    driver_lora_config.coding_rate,
    driver_lora_config.sync_word,
    driver_lora_config.power,
    driver_lora_config.preamble_length,
    driver_lora_config.tcxo_voltage,
    driver_lora_config.use_reg_ldo
  );
  driver_lora_config.lora_obj.setDio1Action(int_dio1);
  driver_lora_config.lora_obj.startReceive();
}

// Returns true if a new transmission is possible.
bool hw_lora_ready() { return (!driver_lora_config.action_complete) && (!driver_lora_config.transmit_mode); }

// Transmits the given buffer or byte, does nothing if LoRa not ready.
void hw_lora_transmit_buffer(uint8_t* buffer, uint32_t size) {
  if (!hw_lora_ready()) return;
  driver_lora_config.transmit_mode = true;
  driver_lora_config.action_complete = false;
  driver_lora_config.lora_obj.startTransmit(buffer, size);
}
void hw_lora_transmit_byte(uint8_t byte) {
  hw_lora_transmit_buffer(&byte, 1);
}

// Returns true if a transmission/reception is complete and hasn't been cleared yet.
bool hw_lora_transmit_pending() { return driver_lora_config.action_complete && driver_lora_config.transmit_mode; }
bool hw_lora_receive_pending() { return driver_lora_config.action_complete && !driver_lora_config.transmit_mode; }

// Clears transmission flags and sets correct module state to accept new receptions (default)
// or start new transmissions through hw_lora_transmit_...
//
// Finishing reception also outputs bytes read into buffer given.
void hw_lora_transmit_finish() {
  // Requires putting into receive mode again as default.
  driver_lora_config.lora_obj.finishTransmit();
  driver_lora_config.transmit_mode = false;
  driver_lora_config.action_complete = false;
  driver_lora_config.lora_obj.startReceive();
}
uint32_t hw_lora_receive_finish(uint8_t* buffer, uint32_t buffer_size) {
  // Packet RX.
  uint32_t bytes_read = driver_lora_config.lora_obj.getPacketLength();
  if (bytes_read > buffer_size) bytes_read = buffer_size;
  driver_lora_config.lora_obj.readData(buffer, bytes_read);

  // Already in receive mode if packet RX complete, just clear the flag.
  driver_lora_config.action_complete = false;

  return bytes_read;
}