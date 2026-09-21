/* -------------------------------------------------------------------------- */
/*                               driver_gpio.hpp                              */
/* -------------------------------------------------------------------------- */
// State-independent interface for functions that run directly off of GPIOS,
// following hw_... naming convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* -------------------------------- Interface ------------------------------- */
// Generic.
void hw_gpio_init() {
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
void hw_gpio_blink(uint32_t count, uint32_t delay_ms) {
  for (uint32_t i = 0; i < count; i++) {
    digitalWrite(PN_MCU_LED, HIGH); delay(delay_ms);
    digitalWrite(PN_MCU_LED, LOW); delay(delay_ms);
  }
}

// PYRO.
bool hw_gpio_pyro1_cont() { return digitalRead(PN_PYRO1_SENS) == HIGH; }
bool hw_gpio_pyro2_cont() { return digitalRead(PN_PYRO2_SENS) == HIGH; }
bool hw_gpio_pyro3_cont() { return digitalRead(PN_PYRO3_SENS) == HIGH; }
bool hw_gpio_pyro4_cont() { return digitalRead(PN_PYRO4_SENS) == HIGH; }
void hw_gpio_pyro1_fire() { digitalWrite(PN_PYRO1_ENA, HIGH); }
void hw_gpio_pyro2_fire() { digitalWrite(PN_PYRO2_ENA, HIGH); }
void hw_gpio_pyro3_fire() { digitalWrite(PN_PYRO3_ENA, HIGH); }
void hw_gpio_pyro4_fire() { digitalWrite(PN_PYRO4_ENA, HIGH); }
void hw_gpio_pyro1_disable() { digitalWrite(PN_PYRO1_ENA, LOW); }
void hw_gpio_pyro2_disable() { digitalWrite(PN_PYRO2_ENA, LOW); }
void hw_gpio_pyro3_disable() { digitalWrite(PN_PYRO3_ENA, LOW); }
void hw_gpio_pyro4_disable() { digitalWrite(PN_PYRO4_ENA, LOW); }
void hw_gpio_pyro_all_fire() {
  hw_gpio_pyro1_fire();
  hw_gpio_pyro2_fire();
  hw_gpio_pyro3_fire();
  hw_gpio_pyro4_fire();
}
void hw_gpio_pyro_all_disable() {
  hw_gpio_pyro1_disable();
  hw_gpio_pyro2_disable();
  hw_gpio_pyro3_disable();
  hw_gpio_pyro4_disable();
}

// Battery.
float hw_bat_volt() { return (float)analogRead(PN_BAT_VOLT) * 0.004833984375; }