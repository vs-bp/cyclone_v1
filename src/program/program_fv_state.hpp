#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateFV {
    // Timing.
    float time_current = 0.0f;              // seconds.
    float time_delta = 0.0f;                // seconds.
    // Event flags.
    bool flag_debug = false;
    bool flag_launch = false;
    bool flag_burnout = false;
    bool flag_apogee = false;
    
    // Fundamental state.
    vec3 ab = vec3(0.0f, 0.0f, 0.0f);       // meters / (second ^ 2).
    vec3 wb = vec3(0.0f, 0.0f, 0.0f);       // radians / second.
    vec3 xe = vec3(0.0f, 0.0f, 0.0f);       // meters.
    vec3 ve = vec3(0.0f, 0.0f, 0.0f);       // meters / second.
    vec3 ae = vec3(0.0f, 0.0f, 0.0f);       // meters / (second ^ 2).
    vec4 q = vec4(1.0f, 0.0f, 0.0f, 0.0f);  // unitless.
    vec3 eul_curr = vec3(0.0f, 0.0f, 0.0f); // radians.
    vec3 eul_rate = vec3(0.0f, 0.0f, 0.0f); // radians.
    mat3 dcmbe = mat3(1.0f);                // unitless.
    mat3 dcmeb = mat3(1.0f);                // unitless.

    // Altitude variables.
    float baro_curr = 0.0f;                 // meters.
    float baro_rate = 0.0f;                 // meters / second.

    // Complementary filter estimates.
    const float cpl_altitude_factor = 0.95f; // unitless.
    const float cpl_velocity_factor = 0.05f; // unitless.
    const float cpl_ab_threshold = 1.0f * g; // meters / (second ^ 2)
    float cpl_altitude = 0.0f;               // meters.
    float cpl_velocity = 0.0f;               // meters / second.

    // GPS variables.
    float latitude = 0.0f;                   // degrees.
    float longitude = 0.0f;                  // degrees.
    uint8_t siv = 0;                         // unitless.

    // Control deflections.
    float canard_angle = 0.0f;               // degrees.
};