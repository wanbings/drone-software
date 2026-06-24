#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "LED.h"
#include "MPU6050.h"
// SPI Defines
// We are going to use SPI 0, and allocate it to the following GPIO pins
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   17
#define PIN_SCK  18
#define PIN_MOSI 19


// Data will be copied from src to dst
const char src[] = "Hello, world! (from DMA)";
char dst[count_of(src)];

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
#define DELAY 500

#define BRIGHTNESS_STEP 500
#define MAX_BRIGHTNESS 10000
int main()
{
    stdio_init_all();
    sleep_ms(2000); 

    printf("Initializing peripherals...\n");
    if (!mpu6050_init()) {
        printf("ERROR: MPU6050 initialization failed!\n");
        while (1);
    }

    LED motor1(MOT1_PIN, 5000);
    motor1.init();

    IMUData imu_data;
    uint32_t last_print_time = to_ms_since_boot(get_absolute_time());

    printf("Controls: Press 'u' to increase brightness, 'd' to decrease.\n");
    printf("Starting main control loop...\n");
    
    while(1){
        if(mpu6050_read(imu_data)){
            //sensor data is updated in the background
        }

        int c = getchar_timeout_us(0);
        if (c == 'u') {
            motor1.increase();
        } else if (c == 'd') {
            motor1.decrease();
        }

        uint32_t current_time = to_ms_since_boot(get_absolute_time());
        if (current_time - last_print_time >= 500) {
            printf("Accel X: %6.2f g | Gyro X: %6.2f deg/s", 
                   imu_data.accel_x, imu_data.gyro_x);
            
            
            last_print_time = current_time;
        }
    }

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

    // gpio_init(MOT1_PIN); gpio_init(MOT2_PIN); gpio_init(MOT3_PIN); gpio_init(MOT4_PIN);
    // gpio_set_dir(MOT1_PIN, GPIO_OUT); gpio_set_dir(MOT2_PIN, GPIO_OUT); gpio_set_dir(MOT3_PIN, GPIO_OUT); gpio_set_dir(MOT4_PIN, GPIO_OUT);



    // while (1) {
        
    // }


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
