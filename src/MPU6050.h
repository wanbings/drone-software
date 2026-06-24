#pragma once

#include <cstdint>

// I2C defines
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA 6
#define I2C_SCL 7
#define MPU6050_ADDR 0x68

#define ACCEL_SCALE_FACTOR 8192.0
#define GYRO_SCALE_FACTOR 131.0
#define ACCEL_CONFIG_VALUE 0X08
#define GYRO_CONFIG_VALUE 0X00
#define SAMPLE_RATE_DIV 1

#define I2C_TIMEOUT 50000

struct IMUData {
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
    float temp;
};

// This single call will handle I2C, reset, and configuration registers.
// Returns true if the MPU6050 was successfully identified ("Who Am I" check passed).
bool mpu6050_init();

// Grabs raw values, processes them, and returns true if successful.
bool mpu6050_read(IMUData &data);