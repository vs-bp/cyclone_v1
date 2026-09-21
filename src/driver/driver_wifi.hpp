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
} wifi_config;

/* ------------------------ Interface Implementations ----------------------- */
void hw_wifi_off() {
  wifi_config.udp_obj.stop();
  WiFi.mode(WIFI_OFF);
}

void hw_wifi_on_ap(String ssid, String pass) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);
    wifi_config.udp_obj.begin(wifi_config.udp_local_port);
}

void hw_wifi_on_client(String ssid, String pass) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    wifi_config.udp_obj.begin(wifi_config.udp_local_port);
}

void hw_wifi_tx_string(String text) {
    wifi_config.udp_obj.beginPacket(wifi_config.udp_broadcast_ip, wifi_config.udp_broadcast_port);
    wifi_config.udp_obj.write(CMD_WIFI_LOG);
    wifi_config.udp_obj.print(text);
    wifi_config.udp_obj.endPacket();
}

void hw_wifi_tx_cmd(uint8_t cmd_byte) {
    wifi_config.udp_obj.beginPacket(wifi_config.udp_broadcast_ip, wifi_config.udp_broadcast_port);
    wifi_config.udp_obj.write(cmd_byte);
    wifi_config.udp_obj.endPacket();
}

uint8_t hw_wifi_rx_byte() {
  int udp_bytes = wifi_config.udp_obj.parsePacket();
  if (udp_bytes > 0) { return wifi_config.udp_obj.read(); }
  else return CMD_GENERIC_NONE;
}

void hw_wifi_begin_ota(String board_name) {
    ArduinoOTA.begin();
    ArduinoOTA.setHostname(board_name.c_str());
    ArduinoOTA.onStart([]() { hw_wifi_tx_string("OTA Download started."); });
    ArduinoOTA.onError([](ota_error_t err) { hw_wifi_tx_string("OTA Download error."); });
    ArduinoOTA.onEnd([]() { hw_wifi_tx_string("OTA Download finished."); });
}

void hw_wifi_handle_ota() {
    ArduinoOTA.handle();
}

bool hw_wifi_connected() {
    return WiFi.status() == WL_CONNECTED;
}