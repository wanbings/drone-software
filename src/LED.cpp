#include "LED.h"
#include "hardware/pwm.h"
#include <stdio.h>

LED::LED(uint gpio_pin, int32_t brightness) : _pin(gpio_pin), _brightness(brightness) {}

void LED::init() {
	gpio_set_function(_pin, GPIO_FUNC_PWM);
	_pwm_slice = pwm_gpio_to_slice_num(_pin);
	pwm_config config = pwm_get_default_config();
	
	pwm_config_set_wrap(&config, MAX_DUTY);
    pwm_init(_pwm_slice, &config, false);
	pwm_set_gpio_level(_pin, _brightness);

	pwm_set_enabled(_pwm_slice, true);
}

void LED::set_brightness(int32_t brightness) {
	if (brightness < 0) {
		_brightness = 0;
	} else if (brightness > MAX_DUTY) {
		_brightness = MAX_DUTY;
	} else {
		_brightness = brightness;
	}
	pwm_set_gpio_level(_pin, _brightness);
}

void LED::increase() {
	_brightness += STEP;
	if (_brightness > MAX_DUTY) {
		_brightness = MAX_DUTY;
	}
	pwm_set_gpio_level(_pin, _brightness);
}

void LED::decrease() {
	_brightness -= STEP;
	if (_brightness < 0) {
		_brightness = 0;
	}
	pwm_set_gpio_level(_pin, _brightness);
}

int LED::get_percentage() {
	return (_brightness * 100) / MAX_DUTY;
}