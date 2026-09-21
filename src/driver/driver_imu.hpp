/* -------------------------------------------------------------------------- */
/*                               driver_imu.hpp                               */
/* -------------------------------------------------------------------------- */
// Configuration, state, and interface for the LSM6DSVETR, following naming
// convention and patterns of other interfaces.
/* -------------------------------------------------------------------------- */
#pragma once
#include <core/core.hpp>

/* ---------------------------- State and Config ---------------------------- */
struct IMUConfig {
    const float rate_hz = 960.0f;
    const float rate_seconds = 1.0f / rate_hz;

    const uint32_t max_gs = 4;
    const uint32_t max_dps = 4000;
    const float sensitivity_gs = 1.0f;
    const float sensitivity_dps = 2.0f;

    mat3 dcm_bs = mat3(1.0f);
    vec3 bias_wb = vec3(0.0f);
    vec3 bias_ab = vec3(0.0f);
    vec3 scale_ab = vec3(1.0f);

    float next_read = 0.0f;
    
    LSM6DSV16XSensor obj = LSM6DSV16XSensor(&Wire, 0xD5);
} driver_imu_config;

/* ------------------------------- Structures ------------------------------- */
struct IMUDataSingle {
  vec3 ab; // m/(s^2)
  vec3 wb; // rad/s
};
struct IMUDataFIFO {
  vec4 q0;           // Unitless.
  vec4 q1;           // Unitless.
  vec3 eul;          // Unitless.
  vec3 ab;           // m/(s^2).
  vec3 wb;           // rad/s.
  mat3 dcmbe;        // Unitless.
  mat3 dcmeb;        // Unitless.
  uint32_t wb_count; // Unitless.
  uint32_t ab_count; // Unitless.
};

/* ------------------------ Interface Implementations ----------------------- */
// Reset, initialize accel/gyro, set accel/gyro range, 
// set accel/gyro rate, set FIFO driver_imu_config.
void hw_imu_init() {
  uint8_t status = 0;
  driver_imu_config.obj.begin();
  driver_imu_config.obj.Device_Reset();
  status |= driver_imu_config.obj.Enable_X();
  status |= driver_imu_config.obj.Enable_G();
  status |= driver_imu_config.obj.Set_X_FS(driver_imu_config.max_gs);
  status |= driver_imu_config.obj.Set_G_FS(driver_imu_config.max_dps);
  status |= driver_imu_config.obj.Set_X_ODR(driver_imu_config.rate_hz);
  status |= driver_imu_config.obj.Set_G_ODR(driver_imu_config.rate_hz);
  status |= driver_imu_config.obj.FIFO_Set_X_BDR(driver_imu_config.rate_hz);
  status |= driver_imu_config.obj.FIFO_Set_G_BDR(driver_imu_config.rate_hz);
  status |= driver_imu_config.obj.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);
}

// Returns if at least one reading is on the sensor based off the time of last
// read and the expected read frequency.
bool hw_imu_ready() {
    return seconds_us() > driver_imu_config.next_read;
}

// Integrates all current FIFO data into an output rotation given an input rotation
// and an averaged body acceleration from all readings. 
// (Which assumes the rotation was small and the body axes equal initial body axes throughout)
IMUDataFIFO hw_imu_integrate(vec4 q0) {
    // IMU Integration result variables.
    vec4 q1 = q0;
    vec3 ab = vec3(0.0);
    vec3 wb = vec3(0.0);
    uint16_t wb_count = 0;
    uint16_t ab_count = 0; 

    // Poll packets in IMU FIFO. Integrate them for rotation, and average their acceleration.
    uint16_t num;
    driver_imu_config.obj.FIFO_Get_Num_Samples(&num);
    for (uint16_t i = 0; i < num; i++) {
        uint8_t tag;
        driver_imu_config.obj.FIFO_Get_Tag(&tag);
        switch (tag) {
        case 1: {
            // Read gyro value.
            int32_t gyro[3];
            driver_imu_config.obj.FIFO_Get_G_Axes(gyro);
            const float mdps2rps = pi / 180000.0;
            vec3 wb_curr = driver_imu_config.dcm_bs * vec3(
                (float)gyro[0]*mdps2rps*driver_imu_config.sensitivity_dps + driver_imu_config.bias_wb.x,
                (float)gyro[1]*mdps2rps*driver_imu_config.sensitivity_dps + driver_imu_config.bias_wb.y,
                (float)gyro[2]*mdps2rps*driver_imu_config.sensitivity_dps + driver_imu_config.bias_wb.z
            );
            // Integrate.
            float bp = wb_curr.x;
            float bq = wb_curr.y;
            float br = wb_curr.z;
            q1 = (q1 + vec4(
                (0.5*(-q1.x*bp - q1.y*bq - q1.z*br)) * driver_imu_config.rate_seconds,
                (0.5*( q1.w*bp + q1.y*br - q1.z*bq)) * driver_imu_config.rate_seconds,
                (0.5*( q1.w*bq - q1.x*br + q1.z*bp)) * driver_imu_config.rate_seconds,
                (0.5*( q1.w*br + q1.x*bq - q1.y*bp)) * driver_imu_config.rate_seconds
            )).normalize();
            // Accumulate wb alone.
            wb = wb + wb_curr;
            wb_count++;
            break;
        }
        case 2: { 
            // Read acceleration value.
            int32_t accel[3];
            driver_imu_config.obj.FIFO_Get_X_Axes(accel);
            // Accumulate.
            const float mg2ms = g / 1000.0;
            ab = ab + driver_imu_config.dcm_bs * ((vec3((float)accel[0], (float)accel[1], (float)accel[2]) * driver_imu_config.scale_ab * mg2ms * driver_imu_config.sensitivity_gs) + driver_imu_config.bias_ab);
            ab_count++;
            break; 
            }
        default: { break; } 
        }
    }

    if (wb_count == 0) { wb = vec3(0.0); }
    else { wb = wb / wb_count; }

    if (ab_count == 0) { ab = vec3(0.0); }
    else { ab = ab / ab_count; }

    // Clear FIFO.
    driver_imu_config.obj.FIFO_Reset();

    // Determine euler values from data.
    float qx = q1.x;
    float qy = q1.y;
    float qz = q1.z;
    float qw = q1.w;
    float t1 = 2*(qw*qy - qx*qz);
    vec3 eul = vec3(
        atan2(2*(qw*qx + qy*qz), 1-2*(qx*qx + qy*qy)),
        (-pi/2) + 2*atan2(sqrt(1+t1), sqrt(1-t1)),
        atan2(2*(qw*qz + qx*qy), 1-2*(qy*qy + qz*qz))
    );

    // Set direction cosine matrices from data.
    mat3 dcmbe = mat3(
        vec3(qw*qw + qx*qx - qy*qy - qz*qz, 2.0f*(qx*qy + qw*qz), 2.0f*(qx*qz - qw*qy)),
        vec3(2.0f*(qx*qy - qw*qz), qw*qw - qx*qx + qy*qy - qz*qz, 2.0f*(qy*qz + qw*qx)),
        vec3(2.0f*(qx*qz + qw*qy), 2.0f*(qy*qz - qw*qz), qw*qw - qx*qx - qy*qy + qz*qz)
    );
    mat3 dcmeb = dcmbe.transpose();

    // Set timer to block ready until at least one reading has
    // been collected.
    driver_imu_config.next_read = seconds_us() + driver_imu_config.rate_seconds;

    // Return integrated results.
    IMUDataFIFO output;
    output.q0 = q0;
    output.q1 = q1;
    output.eul = eul;
    output.ab = ab;
    output.wb = wb;
    output.ab_count = ab_count;
    output.wb_count = wb_count;
    return output;
}

// Single FIFO-less data read on IMU.
IMUDataSingle hw_imu_read() {
    // Read raw data.
    int32_t accel[3], gyro[3];
    driver_imu_config.obj.Get_X_Axes(accel);
    driver_imu_config.obj.Get_G_Axes(gyro);

    // Convert data into correct units and write to output struct.
    const float mg2ms = g / 1000.0;
    const float mdps2rps = pi / 180000.0;
    IMUDataSingle output;
    output.ab = driver_imu_config.dcm_bs * ((vec3((float)accel[0], (float)accel[1], (float)accel[2]) * driver_imu_config.scale_ab * mg2ms * driver_imu_config.sensitivity_gs) + driver_imu_config.bias_ab);
    output.wb = driver_imu_config.dcm_bs * ((vec3((float)gyro[0], (float)gyro[1], (float)gyro[2]) * mdps2rps * driver_imu_config.sensitivity_dps) + driver_imu_config.bias_wb);

    // Set timer to block ready until at least one reading has
    // been collected.
    driver_imu_config.next_read = seconds_us() + driver_imu_config.rate_seconds;

    return output;
}

// Sets calibration values for the IMU.
void hw_imu_set_calibration(vec3 wb_bias, vec3 ab_bias, vec3 ab_scale) {
    driver_imu_config.bias_wb = wb_bias;
    driver_imu_config.bias_ab = ab_bias;
    driver_imu_config.scale_ab = ab_scale;
}