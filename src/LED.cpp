#include "LED.h"
#include "hardware/pwm.h"
#include <stdio.h>

LED::LED(uint gpio_pin, int32_t brightness) : pin(gpio_pin), current_brightness(brightness) {}

void LED::init() {
	gpio_set_function(pin, GPIO_FUNC_PWM);
	slice_num = pwm_gpio_to_slice_num(pin);
	pwm_config config = pwm_get_default_config();
	
	pwm_config_set_wrap(&config, MAX_DUTY);
    pwm_init(slice_num, &config, false);
	pwm_set_gpio_level(pin, current_brightness);

	pwm_set_enabled(slice_num, true);
}

void LED::increase() {
	current_brightness += STEP;
	if (current_brightness > MAX_DUTY) {
		current_brightness = MAX_DUTY;
	}
	pwm_set_gpio_level(pin, current_brightness);
}

void LED::decrease() {
	current_brightness -= STEP;
	if (current_brightness < 0) {
		current_brightness = 0;
	}
	pwm_set_gpio_level(pin, current_brightness);
}

int LED::get_percentage() {
	return (current_brightness * 100) / MAX_DUTY;
}