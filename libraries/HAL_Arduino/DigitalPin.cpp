#include "DigitalPin.h"
#include <Arduino.h>

namespace HAL_Arduino
{
	void DigitalPin::SetMode(uint32_t mode)
	{
		pinMode(_pin, mode);
	}

	void DigitalPin::Write(uint32_t value)
	{
		digitalWrite(_pin, value);
	}
}
