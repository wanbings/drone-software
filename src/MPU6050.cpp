#include "MPU6050.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// mpu6050 register addresses
#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_SMPLR_DIV 0x19
#define WHO_AM_I_REG 0x75

// mpu6050 data registers
#define REG_ACCEL_XOUT_H 0x3B
#define REG_ACCEL_YOUT_H 0x3D
#define REG_ACCEL_ZOUT_H 0x3F
#define REG_GYRO_XOUT_H 0x43
#define REG_GYRO_YOUT_H 0x45
#define REG_GYRO_ZOUT_H 0x47


void mpu6050_reset() {
    std::uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);
    std::uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    sleep_ms(200);
}

void mpu6050_configure() {
    std::uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, accel_config, 2, false);

    std::uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false);

    std::uint8_t sample_rate[] = {REG_SMPLR_DIV, SAMPLE_RATE_DIV};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false);
}

bool mpu6050_read(IMUData &data) {
    std::uint8_t buffer[14];
    std::uint8_t reg = REG_ACCEL_XOUT_H;
    if (i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true) < 0) {
        return false; // I2C failure or device disconnected
    }
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

    int16_t raw_ax = (int16_t)(buffer[0] << 8) | buffer[1];
    int16_t raw_ay = (int16_t)(buffer[2] << 8) | buffer[3];
    int16_t raw_az = (int16_t)(buffer[4] << 8) | buffer[5];
    int16_t t      = (int16_t)(buffer[6] << 8) | buffer[7];
    int16_t raw_gx = (int16_t)(buffer[8] << 8) | buffer[9];
    int16_t raw_gy = (int16_t)(buffer[10] << 8) | buffer[11];
    int16_t raw_gz = (int16_t)(buffer[12] << 8) | buffer[13];

    data.accel_x = (float)raw_ax / ACCEL_SCALE_FACTOR;
    data.accel_y = (float)raw_ay / ACCEL_SCALE_FACTOR;
    data.accel_z = (float)raw_az / ACCEL_SCALE_FACTOR;
    data.temp = (float)t / 340.0f + 36.53f;
    data.gyro_x = (float)raw_gx / GYRO_SCALE_FACTOR;
    data.gyro_y = (float)raw_gy / GYRO_SCALE_FACTOR;
    data.gyro_z = (float)raw_gz / GYRO_SCALE_FACTOR;

    return true;
}

bool mpu6050_init() {
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(100);

    mpu6050_reset();
    mpu6050_configure();
    
    std::uint8_t who_am_i = 0;
    std::uint8_t reg = WHO_AM_I_REG;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    
    return (who_am_i == 0x68);
}