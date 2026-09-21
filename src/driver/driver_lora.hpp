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
} lora_config;

/* ------------------------ Interface Implementations ----------------------- */
// Init function defines DIO1 interrupt as just setting action complete flag to true.
// Radio starts in receive mode.
ICACHE_RAM_ATTR void int_dio1() { lora_config.action_complete = true; }
void hw_lora_init() {
  pinMode(PN_SPI_SCK, OUTPUT);
  pinMode(PN_SPI_MISO, INPUT);
  pinMode(PN_SPI_MOSI, OUTPUT);
  pinMode(PN_RF_NSS, OUTPUT);
  lora_config.spi_obj.begin(PN_SPI_SCK, PN_SPI_MISO, PN_SPI_MOSI, PN_RF_NSS);
  lora_config.lora_obj.begin(
    lora_config.frequency,
    lora_config.bandwidth,
    lora_config.spreading_factor,
    lora_config.coding_rate,
    lora_config.sync_word,
    lora_config.power,
    lora_config.preamble_length,
    lora_config.tcxo_voltage,
    lora_config.use_reg_ldo
  );
  lora_config.lora_obj.setDio1Action(int_dio1);
  lora_config.lora_obj.startReceive();
}

// Returns true if a new transmission is possible.
bool hw_lora_ready() { return (!lora_config.action_complete) && (!lora_config.transmit_mode); }

// Transmits the given buffer, does nothing if LoRa not ready.
void hw_lora_transmit(uint8_t* buffer, uint32_t size) {
  if (!hw_lora_ready()) return;
  lora_config.transmit_mode = true;
  lora_config.action_complete = false;
  lora_config.lora_obj.startTransmit(buffer, size);
}

// Returns true if a transmission of said type is complete and hasn't been cleared yet.
bool hw_lora_transmit_action_complete() { return lora_config.action_complete && lora_config.transmit_mode; }
bool hw_lora_receive_action_complete() { return lora_config.action_complete && !lora_config.transmit_mode; }

// Clears transmission complete so above functions return false.
// TODO.
void hw_lora_clear_action() { lora_config.action_complete = false; }