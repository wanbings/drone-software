#include <stdio.h>
#include <cmath>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/watchdog.h"
#include "hardware/uart.h"
#include "LED.h"
#include "MPU6050.h"
#include "PID.h"
#include "pid_config.h"
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
    uint32_t last_print = 0;
    stdio_init_all();
    sleep_ms(2000); 

    printf("Initializing peripherals...\n");
    if (!mpu6050_init()) {
        printf("ERROR: MPU6050 initialization failed!\n");
        while (1);
    }

    // Rates (Inner) Loops - 1kHz - 8kHz
    PID rollRatePID(Config::DT, Config::MAX_LIMIT, Config::MIN_LIMIT, Config::KP_ROLL_RATE, Config::KD_ROLL_RATE, Config::KI_ROLL_RATE, Config::FILTER_CUTOFF);
    PID pitchRatePID(Config::DT, Config::MAX_LIMIT, Config::MIN_LIMIT, Config::KP_PITCH_RATE, Config::KD_PITCH_RATE, Config::KI_PITCH_RATE, Config::FILTER_CUTOFF);
    PID yawRatePID(Config::DT,   Config::MAX_LIMIT, Config::MIN_LIMIT, Config::KP_YAW_RATE,   Config::KD_YAW_RATE,   Config::KI_YAW_RATE,   Config::FILTER_CUTOFF);

    // Angle (Outer) Loops - 100Hz - 250Hz (Optional, for auto-level stabilization mode)
    PID rollAnglePID(Config::DT_SLOW,  Config::MAX_RATE_LIMIT, Config::MIN_RATE_LIMIT, Config::KP_ROLL_ANGLE,  0.0f, 0.0f);
    PID pitchAnglePID(Config::DT_SLOW, Config::MAX_RATE_LIMIT, Config::MIN_RATE_LIMIT, Config::KP_PITCH_ANGLE, 0.0f, 0.0f);
    
    LED motor1(MOT1_PIN, 0); motor1.init();
    LED motor2(MOT2_PIN, 0); motor2.init();
    LED motor3(MOT3_PIN, 0); motor3.init();
    LED motor4(MOT4_PIN, 0); motor4.init();

    IMUData imu_data;

    //estimation variables
    float current_roll = 0.0f;
    float current_pitch = 0.0f;
    float current_yaw = 0.0f;

    // Flight Targets (0.0f commands the drone to stay perfectly level)
    float target_roll_angle = 0.0f; 
    float target_pitch_angle = 0.0f;
    float target_yaw_rate = 0.0f;

    float throttle = 5000.0f; // 50% hover throttle baseline (out of 10000)
    int outer_loop_counter = 0;
    
    float target_roll_rate = 0.0f;
    float target_pitch_rate = 0.0f;

    // Timing Variables (RP2040 absolute time in microseconds)
    absolute_time_t next_loop_time = get_absolute_time();

    printf("Starting main control loop...\n");

    while(1) {
        busy_wait_until(next_loop_time);
        next_loop_time = delayed_by_us(next_loop_time, 1000);

        if (!mpu6050_read(imu_data)) {
            // If we print this too fast, it will flood the serial monitor. 
            // We'll limit it to once every 500ms.
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - last_print >= 500) {
                printf("WARNING: MPU6050 read failed! Skipping loop.\n");
                last_print = now;
            }
            continue; // This skips everything below!
        }

        // 2. Complementary Filter (Fusing Gyro and Accel for actual physical angles)
        float roll_acc = atan2f(imu_data.accel_y, imu_data.accel_z) * 57.29578f;
        float pitch_acc = atan2f(-imu_data.accel_x, sqrtf(imu_data.accel_y * imu_data.accel_y + imu_data.accel_z * imu_data.accel_z)) * 57.29578f;

        // Fused Angle = 98% (gyro integrated) + 2% (static accelerometer angle)
        current_roll = 0.98f * (current_roll + imu_data.gyro_x * Config::DT) + 0.02f * roll_acc;
        current_pitch = 0.98f * (current_pitch + imu_data.gyro_y * Config::DT) + 0.02f * pitch_acc;
        current_yaw += imu_data.gyro_z * Config::DT; // Directly integrate gyro for heading

        // 3. Slow Outer Loop (Attitude/Angle Control - Runs at 200Hz)
        // Since the outer loop runs at 200Hz (every 5ms), execute it once every 5 iterations of the 1kHz loop.
        outer_loop_counter++;
        if (outer_loop_counter >= 5) {
            outer_loop_counter = 0;
            
            // Calculates desired rotation rates based on current angle errors
            target_roll_rate = rollAnglePID.calculate(target_roll_angle, current_roll);
            target_pitch_rate = pitchAnglePID.calculate(target_pitch_angle, current_pitch);
        }

        // 4. Fast Inner Loop (Rate Control - Runs at 1kHz)
        // Calculates control output adjustments to reach target angular speeds
        float roll_adjust  = rollRatePID.calculate(target_roll_rate,  imu_data.gyro_x);
        float pitch_adjust = pitchRatePID.calculate(target_pitch_rate, imu_data.gyro_y);
        float yaw_adjust   = yawRatePID.calculate(target_yaw_rate,   imu_data.gyro_z);

        // 5. Motor Mixing (X Configuration)
        // Distribute PID corrections to the 4 motor LED pins
        float m1_output = throttle - roll_adjust - pitch_adjust + yaw_adjust; // Front Right (CCW)
        float m2_output = throttle - roll_adjust + pitch_adjust - yaw_adjust; // Rear Right (CW)
        float m3_output = throttle + roll_adjust + pitch_adjust + yaw_adjust; // Rear Left (CCW)
        float m4_output = throttle + roll_adjust - pitch_adjust - yaw_adjust; // Front Left (CW)

        // 6. Actuator Clamping (Keep within 0 to MAX_BRIGHTNESS)
        if (m1_output < 0.0f) m1_output = 0.0f; if (m1_output > MAX_BRIGHTNESS) m1_output = MAX_BRIGHTNESS;
        if (m2_output < 0.0f) m2_output = 0.0f; if (m2_output > MAX_BRIGHTNESS) m2_output = MAX_BRIGHTNESS;
        if (m3_output < 0.0f) m3_output = 0.0f; if (m3_output > MAX_BRIGHTNESS) m3_output = MAX_BRIGHTNESS;
        if (m4_output < 0.0f) m4_output = 0.0f; if (m4_output > MAX_BRIGHTNESS) m4_output = MAX_BRIGHTNESS;



        // 7. Write to "Motors" (LEDs)
        motor1.set_brightness((int)m1_output);
        motor2.set_brightness((int)m2_output);
        motor3.set_brightness((int)m3_output);
        motor4.set_brightness((int)m4_output);

        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print >= 500) {
            printf("Loop active! Roll Angle: %0.1f | M1 PWM: %d | M3 PWM: %d\n", 
                   current_roll, (int)m1_output, (int)m3_output);
            last_print = now;
        }
    }
    // uint32_t last_print_time = to_ms_since_boot(get_absolute_time());

    // printf("Controls: Press 'u' to increase brightness, 'd' to decrease.\n");
    // printf("Starting main control loop...\n");
    
    // while(1){
    //     if(mpu6050_read(imu_data)){
    //         //sensor data is updated in the background
    //     }

    //     int c = getchar_timeout_us(0);
    //     if (c == 'u') {
    //         motor1.increase();
    //     } else if (c == 'd') {
    //         motor1.decrease();
    //     }

    //     uint32_t current_time = to_ms_since_boot(get_absolute_time());
    //     if (current_time - last_print_time >= 500) {
    //         printf("Accel X: %6.2f g | Gyro X: %6.2f deg/s", 
    //                imu_data.accel_x, imu_data.gyro_x);
            
            
    //         last_print_time = current_time;
    //     }
    // }

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
