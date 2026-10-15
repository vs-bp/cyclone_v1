/* -------------------------------------------------------------------------- */
/*                               driver_lora.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the lora module, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <driver/m_base.hpp>

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
} hwm_config_lora;

/* ------------------------ Interface Implementations ----------------------- */
// Init function defines DIO1 interrupt as just setting action complete flag to true.
// Radio starts in receive mode.
ICACHE_RAM_ATTR void int_dio1() { hwm_config_lora.action_complete = true; }
void hwm_lora_init() {
  pinMode(PN_SPI_SCK, OUTPUT);
  pinMode(PN_SPI_MISO, INPUT);
  pinMode(PN_SPI_MOSI, OUTPUT);
  pinMode(PN_RF_NSS, OUTPUT);
  hwm_config_lora.spi_obj.begin(PN_SPI_SCK, PN_SPI_MISO, PN_SPI_MOSI, PN_RF_NSS);
  hwm_config_lora.lora_obj.begin(
    hwm_config_lora.frequency,
    hwm_config_lora.bandwidth,
    hwm_config_lora.spreading_factor,
    hwm_config_lora.coding_rate,
    hwm_config_lora.sync_word,
    hwm_config_lora.power,
    hwm_config_lora.preamble_length,
    hwm_config_lora.tcxo_voltage,
    hwm_config_lora.use_reg_ldo
  );
  hwm_config_lora.lora_obj.setDio1Action(int_dio1);
  hwm_config_lora.lora_obj.startReceive();
}

// Returns true if a new transmission is possible.
bool hwm_lora_ready() { return (!hwm_config_lora.action_complete) && (!hwm_config_lora.transmit_mode); }

// Transmits the given buffer or byte, does nothing if LoRa not ready.
void hwm_lora_transmit_buffer(uint8_t* buffer, uint32_t size) {
  if (!hwm_lora_ready()) return;
  hwm_config_lora.transmit_mode = true;
  hwm_config_lora.action_complete = false;
  hwm_config_lora.lora_obj.startTransmit(buffer, size);
}
void hwm_lora_transmit_byte(uint8_t byte) {
  hwm_lora_transmit_buffer(&byte, 1);
}

// Returns true if a transmission/reception is complete and hasn't been cleared yet.
bool hwm_lora_transmit_pending() { return hwm_config_lora.action_complete && hwm_config_lora.transmit_mode; }
bool hwm_lora_receive_pending() { return hwm_config_lora.action_complete && !hwm_config_lora.transmit_mode; }

// Clears transmission flags and sets correct module state to accept new receptions (default)
// or start new transmissions through hwm_lora_transmit_...
//
// Finishing reception also outputs bytes read into buffer given.
void hwm_lora_transmit_finish() {
  // Requires putting into receive mode again as default.
  hwm_config_lora.lora_obj.finishTransmit();
  hwm_config_lora.transmit_mode = false;
  hwm_config_lora.action_complete = false;
  hwm_config_lora.lora_obj.startReceive();
}
uint32_t hwm_lora_receive_finish(uint8_t* buffer, uint32_t buffer_size) {
  // Packet RX.
  uint32_t bytes_read = hwm_config_lora.lora_obj.getPacketLength();
  if (bytes_read > buffer_size) bytes_read = buffer_size;
  hwm_config_lora.lora_obj.readData(buffer, bytes_read);

  // Already in receive mode if packet RX complete, just clear the flag.
  hwm_config_lora.action_complete = false;

  return bytes_read;
}