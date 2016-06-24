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
			DigitalPin(digitalPin),
			_uart(uart)
		{
		}

		uint32_t Available();
		uint32_t Read();
		void Begin(uint32_t baud);
		void Clear();

	private:
		UARTClass* _uart;

		uint32_t _currentBaud;
	};
}

#endif // !_HAL_Arduino_UartRx
