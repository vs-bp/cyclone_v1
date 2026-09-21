#pragma once
#include <driver/driver.hpp>

/* -------------------------------------------------------------------------- */
/*                                Global State                                */
/* -------------------------------------------------------------------------- */
struct ProgramStateFV {
    // Fundamental state.
    vec3 xe = vec3(0.0f, 0.0f, 0.0f);
    vec3 ve = vec3(0.0f, 0.0f, 0.0f);
    vec3 ab = vec3(0.0f, 0.0f, 0.0f);
    vec3 ae = vec3(0.0f, 0.0f, 0.0f); 
    vec4 q = vec4(1.0f, 0.0f, 0.0f, 0.0f);
    vec3 eul_curr = vec3(0.0f, 0.0f, 0.0f);
    vec3 eul_rate = vec3(0.0f, 0.0f, 0.0f);
    mat3 dcmbe = mat3(1.0f);
    mat3 dcmeb = mat3(1.0f);

    // Altitude variables.
    float baro_curr = 0.0f;
    float baro_rate = 0.0f;

    // Complementary filter estimates.
    const float cpl_altitude_factor = 0.95f;
    const float cpl_velocity_factor = 0.05f;
    const float cpl_ab_threshold = 1.0f * g;
    float cpl_altitude = 0.0f;
    float cpl_velocity = 0.0f;

    // GPS variables.
    float latitude = 0.0f;
    float longitude = 0.0f;
    uint8_t siv = 0;
};