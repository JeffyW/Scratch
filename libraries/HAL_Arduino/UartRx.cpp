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

// Serial / USART registers
// UBRR0H - Upper 8 bits of the baud rate value
// UBRR0L - Lower 8 bits of the baud rate value
//    Note that the U2X0 and clock must be taken account for.
// UCSR0C - Character size?
//    UMSEL01:0(7,6) - Mode of communication.  00 is Async USART
//    UPM01:0(5,4)   - Parity behavior.  00 is disabled
//    USBS0(3)       - Bit singlaling - 0 is 1-bit stop, 1 is 2-bit stop
//    UCSZ00
//    UCSZ01:0(2,1)  - Along with UCSZ02(2) in UCSR0B sets number of data bits.
//    UCSZ10
//    (0) - Clock polarity.  0 means TX Rising, RX Falling
// UCSR0B - interrupts
//    RXEN0  4 - Receive
//    TXEN0  3 - Transmit
//    RXCIE0 7 - Receive interrupt
