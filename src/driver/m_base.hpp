/* -------------------------------------------------------------------------- */
/*                               driver_gpio.hpp                              */
/* -------------------------------------------------------------------------- */
// State-independent interface for functions that run directly off of GPIOS,
// following hwm_... naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/_core.hpp>

/* ---------------------------------- Pins ---------------------------------- */
#define PN_SPI_SCK 15
#define PN_SPI_MOSI 16
#define PN_SPI_MISO 17
#define PN_RF_NSS 6
#define PN_RF_NRST 7
#define PN_RF_DIO1 3
#define PN_RF_BUSY 10
#define PN_RF_SW 11
#define PN_GPIO3 5
#define PN_GPIO4 4
#define PN_GPIO5 8
#define PN_GPIO6 18
#define PN_PYRO1_ENA 38
#define PN_PYRO2_ENA 39
#define PN_PYRO3_ENA 41
#define PN_PYRO4_ENA 2
#define PN_PYRO1_SENS 12
#define PN_PYRO2_SENS 40
#define PN_PYRO3_SENS 42
#define PN_PYRO4_SENS 1
#define PN_MCU_LED 9
#define PN_BAT_VOLT 13
#define PN_I2C_SCL 21
#define PN_I2C_SDA 14

/* ---------------------------- State and Config ---------------------------- */
struct GPIOConfig {
  float timer_pyro1 = -1.0f;
  float timer_pyro2 = -1.0f;
  float timer_pyro3 = -1.0f;
  float timer_pyro4 = -1.0f;
  bool state_pyro1 = false;
  bool state_pyro2 = false;
  bool state_pyro3 = false;
  bool state_pyro4 = false;
} hwm_config_gpio;

/* -------------------------------- Interface ------------------------------- */
// Generic.
void hwm_gpio_init() {
  pinMode(PN_MCU_LED, OUTPUT);
  pinMode(PN_BAT_VOLT, INPUT);
  pinMode(PN_PYRO1_ENA, OUTPUT);
  pinMode(PN_PYRO2_ENA, OUTPUT);
  pinMode(PN_PYRO3_ENA, OUTPUT);
  pinMode(PN_PYRO4_ENA, OUTPUT);
  digitalWrite(PN_PYRO1_ENA, LOW);
  digitalWrite(PN_PYRO2_ENA, LOW);
  digitalWrite(PN_PYRO3_ENA, LOW);
  digitalWrite(PN_PYRO4_ENA, LOW);
  pinMode(PN_PYRO1_SENS, INPUT);
  pinMode(PN_PYRO2_SENS, INPUT);
  pinMode(PN_PYRO3_SENS, INPUT);
  pinMode(PN_PYRO4_SENS, INPUT);
  Wire.begin(PN_I2C_SDA, PN_I2C_SCL, 400000);
}

// LED.
void hwm_gpio_blink(uint32_t count, uint32_t delay_ms) {
  for (uint32_t i = 0; i < count; i++) {
    digitalWrite(PN_MCU_LED, HIGH); delay(delay_ms);
    digitalWrite(PN_MCU_LED, LOW); delay(delay_ms);
  }
}

// PYRO.
bool hwm_gpio_pyro1_cont() { return digitalRead(PN_PYRO1_SENS) == HIGH; }
bool hwm_gpio_pyro2_cont() { return digitalRead(PN_PYRO2_SENS) == HIGH; }
bool hwm_gpio_pyro3_cont() { return digitalRead(PN_PYRO3_SENS) == HIGH; }
bool hwm_gpio_pyro4_cont() { return digitalRead(PN_PYRO4_SENS) == HIGH; }
void hwm_gpio_pyro1_fire(float time) { digitalWrite(PN_PYRO1_ENA, HIGH); hwm_config_gpio.timer_pyro1 = seconds_us() + time; hwm_config_gpio.state_pyro1 = true; }
void hwm_gpio_pyro2_fire(float time) { digitalWrite(PN_PYRO2_ENA, HIGH); hwm_config_gpio.timer_pyro2 = seconds_us() + time; hwm_config_gpio.state_pyro2 = true; }
void hwm_gpio_pyro3_fire(float time) { digitalWrite(PN_PYRO3_ENA, HIGH); hwm_config_gpio.timer_pyro3 = seconds_us() + time; hwm_config_gpio.state_pyro3 = true; }
void hwm_gpio_pyro4_fire(float time) { digitalWrite(PN_PYRO4_ENA, HIGH); hwm_config_gpio.timer_pyro4 = seconds_us() + time; hwm_config_gpio.state_pyro4 = true; }
void hwm_gpio_pyro1_disable() { digitalWrite(PN_PYRO1_ENA, LOW); hwm_config_gpio.state_pyro1 = false; }
void hwm_gpio_pyro2_disable() { digitalWrite(PN_PYRO2_ENA, LOW); hwm_config_gpio.state_pyro2 = false; }
void hwm_gpio_pyro3_disable() { digitalWrite(PN_PYRO3_ENA, LOW); hwm_config_gpio.state_pyro3 = false; }
void hwm_gpio_pyro4_disable() { digitalWrite(PN_PYRO4_ENA, LOW); hwm_config_gpio.state_pyro4 = false; }
void hwm_gpio_pyro_check_timers() {
  float time = seconds_us();
  if (hwm_config_gpio.state_pyro1) { if (time > hwm_config_gpio.timer_pyro1) hwm_gpio_pyro1_disable(); }
  if (hwm_config_gpio.state_pyro2) { if (time > hwm_config_gpio.timer_pyro2) hwm_gpio_pyro2_disable(); }
  if (hwm_config_gpio.state_pyro3) { if (time > hwm_config_gpio.timer_pyro3) hwm_gpio_pyro3_disable(); }
  if (hwm_config_gpio.state_pyro4) { if (time > hwm_config_gpio.timer_pyro4) hwm_gpio_pyro4_disable(); }
}

// Battery.
float hwm_bat_volt() { return (float)analogRead(PN_BAT_VOLT) * 0.004833984375; }