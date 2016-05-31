#ifndef _HAL_Arduino_UartRx
#define _HAL_Arduino_UartRx

#include <HAL/UartRx.h>
#include <UARTClass.h>
#include "DigitalPin.h"

namespace HAL_Arduino
{
	class UartRx : public DigitalPin, public HAL::UartRx
	{
	public:
		UartRx(uint32_t digitalPin, UARTClass* uart)
			:
			HAL_Arduino::DigitalPin::DigitalPin(digitalPin),
			_uart(uart)
		{
		}

		uint32_t Available();
		uint32_t Read();
		void Begin(uint32_t baud);
		void Clear();

		void SetMode(uint32_t mode) { HAL_Arduino::DigitalPin::SetMode(mode); }
		void Write(uint32_t value) { HAL_Arduino::DigitalPin::Write(value); }

	private:
		UARTClass* _uart;

		uint32_t _currentBaud;
	};
}

#endif // !_HAL_Arduino_UartRx
