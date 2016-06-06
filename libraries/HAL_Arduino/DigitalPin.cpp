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

	uint32_t DigitalPin::Read()
	{
		return digitalRead(_pin);
	}
}

// Registers for SAM3X series
// There are 5 ports available, each is 32bits, mapped to the pins documented in variant.cpp
//   PIOx where x is A,B,C,D,E
// digitalPinToPort can be used to find which port a pin is mapped to
// digitalPinToBitMask can be used to find which bit in a port a pin is mapped to
//
//Pio *rxport = digitalPinToPort(_pin);
//uint32_t rxbitmask = digitalPinToBitMask(_pin);
//
// There are several registers for each port that read and write
// See Component_pio for reference
// PIO_PER - Write 1's here to override other peripherals and allow GPIO use for pins
// PIO_OER - Write 1's here to set pins as OUTPUT
// PIO_ODR - Write 1's here to set pins as INPUT

// PIO_SODR - Write 1's here to set output pins as HIGH
// PIO_CODR - Write 1's here to set output pins as LOW
// PIO_PDSR - Read the state of the port
// PIO_ODSR - Status register (I think you can read and write here)

// PIO_IER - Write 1's here to enable interrupts
// PIO_IDR - Write 1's here to disable interrups

// PIO_PUDR - write 1's here to switch OFF internal pull-up for pins (INPUT)
// PIO_PUER - write 1's here to switch ON internall pull-up for pins (INPUT_PULLUP)
// OTHERS for stuff

// These are registers for the PORTs
// Each bit in the register maps to a port number
// Therefore, PORTD B0001 maps to pin 0 on Port D
// DDRx		- Data Direction Register.  Sets INPUT/OUTPUT.  Read/Write
// PORTx	- Data Register.  Sets state of pin as HIGH/LOW.  Read/Write
//    Is that just for OUTPUT or does it also set Pullup/Pulldown?
// PINx		- Port Input Register.  Reads current state of pin.  Read

