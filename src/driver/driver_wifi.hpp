/* -------------------------------------------------------------------------- */
/*                               driver_wifi.hpp                              */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the wifi module, following
// naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#ifndef MAIN_DRIVER_WIFI
#define MAIN_DRIVER_WIFI
#include <core.hpp>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>

/* ---------------------------- State and Config ---------------------------- */
struct WiFiConfig {
    WiFiUDP udp_obj;

    const char* udp_broadcast_ip = "255.255.255.255";
    const uint32_t udp_broadcast_port = 3333;
    const uint32_t udp_local_port = 3333;
} wifi_config;

/* -------------------------- Interface Definitions ------------------------- */
void hw_wifi_off();
void hw_wifi_on_ap(String ssid, String pass);
void hw_wifi_on_client(String ssid, String pass);
void hw_wifi_tx_string(String text);
void hw_wifi_begin_ota(String board_name);
void hw_wifi_handle_ota();

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
    // TODO wifi_config.udp_obj.write(byte);
    wifi_config.udp_obj.endPacket();
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

#endif