#ifndef _HAL_Arduino_UartRx
#define _HAL_Arduino_UartRx

#include <HAL/UartRx.h>
#include "DigitalPin.h"

#include <HardwareSerial.h>

namespace HAL_Arduino
{
	class UartRx : public HAL_Arduino::DigitalPin, public virtual HAL::UartRx
	{
	public:
		UartRx(uint32_t digitalPin, HardwareSerial* uart) :
			DigitalPin(digitalPin),
			_uart(uart)
		{
		}

		uint32_t Available() override;
		uint32_t Read() override;
		void Begin(uint32_t baud) override;
		void Clear() override;

	private:
		HardwareSerial* _uart;
		uint32_t _currentBaud;
	};
}

#endif // !_HAL_Arduino_UartRx
