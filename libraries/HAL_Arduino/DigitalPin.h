#ifndef _HAL_Arduino_DigitalPin
#define _HAL_Arduino_DigitalPin

#include <stdint.h>
#include <HAL/DigitalPin.h>

namespace HAL_Arduino
{
	class DigitalPin : public HAL::DigitalPin
	{
	public:
		DigitalPin(uint32_t pin)
			: _pin(pin)
		{
		}

		void SetMode(uint32_t mode);
		void Write(uint32_t value);

	protected:
		uint32_t _pin;
	};
}

#endif // !_HAL_Arduino_DigitalPin
