#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"

// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c1
#define I2C_SDA 6
#define I2C_SCL 7
#define MPU6050_ADDR 0x68

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

#define ACCEL_SCALE_FACTOR 8192.0
#define GYRO_SCALE_FACTOR 131.0
#define ACCEL_CONFIG_VALUE 0X08
#define GYRO_CONFIG_VALUE 0X00
#define SAMPLE_RATE_DIV 1

#define I2C_TIMEOUT 50000
bool mpu_reset() {
    uint8_t reset[] = {REG_PWR_MGMT_1, 0x80};

    int result = i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, reset, 2, false, 50000);
    if (result < 0) return false; //failed to communicate safely

    //i2c_write_blocking(I2C_PORT, MPU6050_ADDR, reset, 2, false);
    sleep_ms(200);
    uint8_t wake[] = {REG_PWR_MGMT_1, 0x00};
    //i2c_write_blocking(I2C_PORT, MPU6050_ADDR, wake, 2, false);
    //sleep_ms(200);

    result = i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, wake, 2, false, 50000);
    return (result >= 0);
}
bool mpu6050_configure() {
    uint8_t accel_config[] = {REG_ACCEL_CONFIG, ACCEL_CONFIG_VALUE};
    if (i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, accel_config, 2, false, I2C_TIMEOUT) < 0) return false;

    uint8_t gyro_config[] = {REG_GYRO_CONFIG, GYRO_CONFIG_VALUE};
    if (i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, gyro_config, 2, false, I2C_TIMEOUT) < 0) return false;

    uint8_t sample_rate[] = {REG_SMPLR_DIV, SAMPLE_RATE_DIV};
    if (i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, sample_rate, 2, false, I2C_TIMEOUT) < 0) return false;

    return true;
}

// void mpu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
//     uint8_t buffer[14];
//     uint8_t reg = REG_ACCEL_XOUT_H;
//     i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
//     i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 14, false);

//     accel[0] = (buffer[0] << 8) | buffer[1];
//     accel[1] = (buffer[2] << 8) | buffer[3];
//     accel[2] = (buffer[4] << 8) | buffer[5];
//     *temp = (buffer[6] << 8) | buffer[7];
//     gyro[0] = (buffer[8] << 8) | buffer[9];
//     gyro[1] = (buffer[10] << 8) | buffer[11];
//     gyro[2] = (buffer[12] << 8) | buffer[13];
// }
// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

bool mpu_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[14];
    uint8_t reg = REG_ACCEL_XOUT_H;
    
    int w_res = i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, &reg, 1, true, I2C_TIMEOUT);
    int r_res = i2c_read_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, buffer, 14, false, I2C_TIMEOUT);

    if (w_res < 0 || r_res < 0) {
        return false; // Communication failed, but did not hang!
    }

    accel[0] = (buffer[0] << 8) | buffer[1];
    accel[1] = (buffer[2] << 8) | buffer[3];
    accel[2] = (buffer[4] << 8) | buffer[5];
    *temp    = (buffer[6] << 8) | buffer[7];
    gyro[0]  = (buffer[8] << 8) | buffer[9];
    gyro[1]  = (buffer[10] << 8) | buffer[11];
    gyro[2]  = (buffer[12] << 8) | buffer[13];
    return true;
}
// UART defines
// By default the stdout UART is `uart0`, so we will use the second one
#define UART_ID uart1
#define BAUD_RATE 115200

// Use pins 4 and 5 for UART1
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// LED pins (in place of motors)
#define MOT1_PIN 3
#define MOT2_PIN 28
#define MOT3_PIN 13
#define MOT4_PIN 20

int main()
{
    stdio_init_all();

    sleep_ms(3000); 
    printf("\n=== Pico MPU6050 Diagnostic Start ===\n");
    // // SPI initialisation. This example will use SPI at 1MHz.
    // spi_init(SPI_PORT, 1000*1000);
    // gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    // gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    // gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    // gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // // Chip select is active-low, so we'll initialise it to a driven-high state
    // gpio_set_dir(PIN_CS, GPIO_OUT);
    // gpio_put(PIN_CS, 1);
    // // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

    gpio_init(MOT1_PIN); gpio_init(MOT2_PIN); gpio_init(MOT3_PIN); gpio_init(MOT4_PIN);
    gpio_set_dir(MOT1_PIN, GPIO_OUT); gpio_set_dir(MOT2_PIN, GPIO_OUT); gpio_set_dir(MOT3_PIN, GPIO_OUT); gpio_set_dir(MOT4_PIN, GPIO_OUT);

    // // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(100);
    

    //reset and configure mpu6050
    //mpu_reset();
    //mpu6050_configure();
    printf("Resetting MPU6050 \n");
    if (!mpu_reset()) {
        printf("Failed to reset MPU6050\n");
    } else {
        mpu6050_configure();

        uint8_t who_am_i = 0;
        uint8_t reg = WHO_AM_I_REG;
        i2c_write_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, &reg, 1, true, 50000);
        i2c_read_timeout_per_char_us(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false, 50000);
        printf("Who am I: 0x%02X\n", who_am_i);
    }
    //i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    //i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &who_am_i, 1, false);
    //printf("Who am I: 0x%02X\n", who_am_i);

    // if (who_am_i != 0x68) {
    //     printf("MPU6050 not found!\n");
    //     while (1);
    // }
    printf("Entering main loop. LEDs should now begin flashing...\n");
    int16_t accel[3], gyro[3], temp;
    bool led_state = false;


    while (1) {
         led_state = !led_state;
        gpio_put(MOT1_PIN, led_state);
        gpio_put(MOT2_PIN, led_state);
        gpio_put(MOT3_PIN, led_state);
        gpio_put(MOT4_PIN, led_state);
        

        if(mpu_read_raw(accel, gyro, &temp)) {
        
        //convert raw values to actual values
        float accel_g[3];
        accel_g[0] = (float)accel[0] / ACCEL_SCALE_FACTOR;
        accel_g[1] = (float)accel[1] / ACCEL_SCALE_FACTOR;
        accel_g[2] = (float)accel[2] / ACCEL_SCALE_FACTOR;

        float gyro_dps[3];
        gyro_dps[0] = (float)gyro[0] / GYRO_SCALE_FACTOR;
        gyro_dps[1] = (float)gyro[1] / GYRO_SCALE_FACTOR;
        gyro_dps[2] = (float)gyro[2] / GYRO_SCALE_FACTOR;

        float temp_c = (float)temp / 340.0f + 36.53f;

        printf("Accel: %.2f %.2f %.2f g, ", accel_g[0], accel_g[1], accel_g[2]);
        printf("Gyro: %.2f %.2f %.2f deg/s, ", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("Temp: %.2f C\n", temp_c);

        } else {
            printf("Failed to read from MPU6050\n");
        }
    
        sleep_ms(500);
    }
    return 0;


    // // Get a free channel, panic() if there are none
    // int chan = dma_claim_unused_channel(true);
    
    // // 8 bit transfers. Both read and write address increment after each
    // // transfer (each pointing to a location in src or dst respectively).
    // // No DREQ is selected, so the DMA transfers as fast as it can.
    
    // dma_channel_config c = dma_channel_get_default_config(chan);
    // channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    // channel_config_set_read_increment(&c, true);
    // channel_config_set_write_increment(&c, true);
    
    // dma_channel_configure(
    //     chan,          // Channel to be configured
    //     &c,            // The configuration we just created
    //     dst,           // The initial write address
    //     src,           // The initial read address
    //     count_of(src), // Number of transfers; in this case each is 1 byte.
    //     true           // Start immediately.
    // );
    
    // // We could choose to go and do something else whilst the DMA is doing its
    // // thing. In this case the processor has nothing else to do, so we just
    // // wait for the DMA to finish.
    // dma_channel_wait_for_finish_blocking(chan);
    
    // // The DMA has now copied our text from the transmit buffer (src) to the
    // // receive buffer (dst), so we can print it out from there.
    // puts(dst);

    // // Watchdog example code
    // // if (watchdog_caused_reboot()) {
    // //     printf("Rebooted by Watchdog!\n");
    // //     // Whatever action you may take if a watchdog caused a reboot
    // // }
    
    // // Enable the watchdog, requiring the watchdog to be updated every 100ms or the chip will reboot
    // // second arg is pause on debug which means the watchdog will pause when stepping through code
    // //watchdog_enable(100, 1);
    
    // // You need to call this function at least more often than the 100ms in the enable call to prevent a reboot
    // //watchdog_update();

    // // Set up our UART
    // uart_init(UART_ID, BAUD_RATE);
    // // Set the TX and RX pins by using the function select on the GPIO
    // // Set datasheet for more information on function select
    // gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    // gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    // // Use some the various UART functions to send out data
    // // In a default system, printf will also output via the default UART
    
    // // Send out a string, with CR/LF conversions
    // uart_puts(UART_ID, " Hello, UART!\n");
    
    // // For more examples of UART use see https://github.com/raspberrypi/pico-examples/tree/master/uart
        
}
