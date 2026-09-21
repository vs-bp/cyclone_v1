/* -------------------------------------------------------------------------- */
/*                               driver_wifi.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the wifi module, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* ---------------------------- State and Config ---------------------------- */
struct WiFiConfig {
    WiFiUDP udp_obj;

    const char* udp_broadcast_ip = "255.255.255.255";
    const uint32_t udp_broadcast_port = 3333;
    const uint32_t udp_local_port = 3333;

    bool ota_initialized = false;
} driver_wifi_config;

/* ------------------------ Interface Implementations ----------------------- */
// Transmits given string with log command byte before.
void hw_wifi_tx_log(String text) {
    driver_wifi_config.udp_obj.beginPacket(driver_wifi_config.udp_broadcast_ip, driver_wifi_config.udp_broadcast_port);
    driver_wifi_config.udp_obj.write(CMD_WIFI_LOG);
    driver_wifi_config.udp_obj.print(text);
    driver_wifi_config.udp_obj.endPacket();
}

// Transmits single-byte command.
void hw_wifi_tx_cmd(uint8_t cmd_byte) {
    driver_wifi_config.udp_obj.beginPacket(driver_wifi_config.udp_broadcast_ip, driver_wifi_config.udp_broadcast_port);
    driver_wifi_config.udp_obj.write(cmd_byte);
    driver_wifi_config.udp_obj.endPacket();
}

// Transmits given buffer as data packet
void hw_wifi_tx_data(uint8_t* data_buffer, uint32_t data_size) {
    driver_wifi_config.udp_obj.beginPacket(driver_wifi_config.udp_broadcast_ip, driver_wifi_config.udp_broadcast_port);
    driver_wifi_config.udp_obj.write(CMD_WIFI_DATA);
    driver_wifi_config.udp_obj.write(data_buffer, data_size);
    driver_wifi_config.udp_obj.endPacket();
}

// Disables all WiFi and UDP functions.
void hw_wifi_off() {
  driver_wifi_config.udp_obj.stop();
  WiFi.mode(WIFI_OFF);
}

// Sets up AP with given name and password.
// Also intitializes UDP and OTA if possible.
void hw_wifi_on_ap(String ssid, String pass, String board_name) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);
    driver_wifi_config.udp_obj.begin(driver_wifi_config.udp_local_port);

    if (!driver_wifi_config.ota_initialized) {
        ArduinoOTA.begin();
        ArduinoOTA.setHostname(board_name.c_str());
        ArduinoOTA.onStart([]() { hw_wifi_tx_log("OTA Download started."); });
        ArduinoOTA.onError([](ota_error_t err) { hw_wifi_tx_log("OTA Download error."); });
        ArduinoOTA.onEnd([]() { hw_wifi_tx_log("OTA Download finished."); });
    }
}

// Simple wrapper over ArduinoOTA.handle() for syntax reasons.
void hw_wifi_handle_ota() {
    ArduinoOTA.handle();
}

// Returns true if connected in station mode.
bool hw_wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}

// Connects to station with given name and password.
// Also intitializes UDP and OTA if possible.
void hw_wifi_on_client(String ssid, String pass, String board_name) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    driver_wifi_config.udp_obj.begin(driver_wifi_config.udp_local_port);

    if (!driver_wifi_config.ota_initialized) {
        ArduinoOTA.begin();
        ArduinoOTA.setHostname(board_name.c_str());
        ArduinoOTA.onStart([]() { hw_wifi_tx_log("OTA Download started."); });
        ArduinoOTA.onError([](ota_error_t err) { hw_wifi_tx_log("OTA Download error."); });
        ArduinoOTA.onEnd([]() { hw_wifi_tx_log("OTA Download finished."); });
        driver_wifi_config.ota_initialized = true;
    }
}

// Receives single-byte command over UDP.
// Returns GENERIC_NONE on no command RX.
uint8_t hw_wifi_rx_byte() {
  int udp_bytes = driver_wifi_config.udp_obj.parsePacket();
  if (udp_bytes > 0) { return driver_wifi_config.udp_obj.read(); }
  else return CMD_GENERIC_NONE;
}

// Receives data from UDP to given buffer, returns number of bytes read.
// Returns 0 if no packets pending.
uint32_t hw_wifi_rx_buffer(uint8_t* buffer, uint32_t buffer_size) {
    uint32_t udp_packet_bytes = driver_wifi_config.udp_obj.parsePacket();
    if (!udp_packet_bytes) return 0;

    return driver_wifi_config.udp_obj.read(buffer, 256);

}