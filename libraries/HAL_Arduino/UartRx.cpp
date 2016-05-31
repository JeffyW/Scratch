#include "UartRx.h"
#include <Arduino.h>

namespace HAL_Arduino
{
	uint32_t UartRx::Available()
	{
		return _uart->available();
	}

	uint32_t UartRx::Read()
	{
		return _uart->read();
	}

	void UartRx::Begin(uint32_t baud)
	{
		_currentBaud = baud;
		_uart->begin(baud);
	}

	void UartRx::Clear()
	{
		// This is overkill, but the only way I can see to clear the buffer.
		_uart->begin(_currentBaud);
	}
}
