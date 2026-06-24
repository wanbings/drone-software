#pragma once

#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/time.h"

//Enumeration for LED modes
enum LEDMode {LEDOff, LEDOn, LEDFade, LEDFadeTo};

class LED {
private:
	uint pin;
	uint slice_num;
	int32_t current_brightness;
	const uint16_t MAX_DUTY = 10000;
	const uint16_t STEP = 500;

public:
	LED(uint gpio_pin, int32_t brightness);
	
	void init();
	void increase();
	void decrease();
	int get_percentage();
};

