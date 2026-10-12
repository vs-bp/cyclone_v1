#pragma once
#include <Arduino.h>

/* -------------------------------------------------------------------------- */
/*                                 Timekeeping                                */
/* -------------------------------------------------------------------------- */
float seconds_us() { return (float)micros() / 1000000.0f; }
float seconds_ms() { return (float)millis() / 1000.0f; }