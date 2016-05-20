/*
Name:		SpektrumTest.ino
Created:	5/11/2016 8:49:30 AM
Author:	jeffy
*/

#include "dsm.h"
#include <stdarg.h>

static void debug(char *fmt, ...){
	char buf[128]; // resulting string limited to 128 chars
	va_list args;
	va_start(args, fmt);
	vsnprintf(buf, 128, fmt, args);
	va_end(args);
	Serial.println(buf);
}

namespace AP_HAL {

	uint32_t micros()
	{
		// Use function provided by libmaple.
		return ::micros();
	}

	uint32_t millis()
	{
		// Use function provided by libmaple.
		return ::millis();
	}

	uint64_t millis64()
	{
		return millis();
	}

	uint64_t micros64()
	{
		// this is slow, but solves the problem with logging uint64_t timestamps
		uint64_t ret = millis();
		ret *= 1000ULL;
		ret += micros() % 1000;
		return ret;
	}

}

#define GPIO_SPEKTRUM_PWR_EN		38 //(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)
#define GPIO_USART1_RX_SPEKTRUM		15 //(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)
#define POWER_SPEKTRUM(_s)		digitalWrite(GPIO_SPEKTRUM_PWR_EN, (_s)) //stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))
#define SPEKTRUM_RX_HIGH(_s)	digitalWrite(GPIO_USART1_RX_SPEKTRUM, (_s)) //stm32_gpiowrite(GPIO_USART1_RX_SPEKTRUM, (_s))
#define SPEKTRUM_RX_AS_UART()		stm32_configgpio(GPIO_USART1_RX)
#define SPEKTRUM_RX_AS_GPIO()		stm32_configgpio(GPIO_USART1_RX_SPEKTRUM)

#define usleep(_s)	delayMicroseconds(_s)
#define dsm_udelay(arg) usleep(arg)
uint16_t  numChannels;
uint16_t values[16];

Pio *rxport = digitalPinToPort(GPIO_USART1_RX_SPEKTRUM);
uint32_t rxbitmask = digitalPinToBitMask(GPIO_USART1_RX_SPEKTRUM);

void setup()
{
	Serial.begin(115200);

	pinMode(GPIO_SPEKTRUM_PWR_EN, OUTPUT);
	////pinMode(Bind_pin, INPUT_PULLUP);
	////if (digitalRead(Bind_pin) == LOW){
		SpektrumBind(); // Place Sat into bind mode to interface with the Spektrum sat
	////}

	//rxport->PIO_PER = rxbitmask; // Disable GPIO
	digitalWrite(GPIO_SPEKTRUM_PWR_EN, HIGH);
	// This line resets all of the Serial3 input, even though we disable it in SpektrumBind
	Serial3.begin(115200); //Uses Serial3 for input as default
}

void loop()
{
	while (Serial3.available() >= 16)
	{

		uint8_t dsm_frame[16];
		uint8_t i;
		for (i = 0; i <= 15; i++)
		{
			dsm_frame[i] = Serial3.read();
		}

		uint16_t num_values = 0;
		if (!dsm_decode(AP_HAL::micros64(), dsm_frame, values, &num_values, numChannels)) {
			debug("WTF");
		}

		debug("%u	%u	%u	%u	%u	%u	%u	%u	%u", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
	}
}

void SpektrumBind(void)
{
	//digitalPinToInterrupt

	// We're using Serial3, which maps to Pin 15, RX3
	// This maps to PORTD5

	unsigned char connected = 0;

	// Connect the power for the Rx to RX_powerpin this is brought
	// high to turn on the Rx.
	delay(5);  // Delay added to work with Orange Receivers
	digitalWrite(GPIO_SPEKTRUM_PWR_EN, HIGH);

	// This is disabling all of the info on Serial3
	rxport->PIO_IDR = rxbitmask; // Disable RX interrupt
	rxport->PIO_PDR = rxbitmask; // Disable GPIO
	//UCSR0B &= ~((1 << RXCIE0) | (1 << RXEN0)); // disable Rx interrupt and USART1 Rx respectively

	rxport->PIO_CODR = rxbitmask; // disable pull-up
	//PORTD &= ~(1 << PORTD5); // disable pull-up

	// Registers for SAM3X series
	// There are 5 ports available, each is 32bits, mapped to the pins documented in variant.cpp
	//   PIOx where x is A,B,C,D,E
	// digitalPinToPort can be used to find which port a pin is mapped to
	// digitalPinToBitMask can be used to find which bit in a port a pin is mapped to
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

	unsigned long time;
	while (time <= 10000) // Wait 10 seconds for spektrum sat connection
	{
		time = millis();
		if (rxport->PIO_PDSR & rxbitmask)
			//if (PIND & (1 << PORTD5))
		{
			connected = 1;
			break;
		}
	}

	if (connected)
	{
		Serial.println("Connected! Bind now!");
		//DDRD |= (1 << DDD5); // Rx as output
		pinMode(GPIO_USART1_RX_SPEKTRUM, OUTPUT);

		delay(90); // Delay after Rx startup

		// === Update 2011-08-18 ===
		// Bind mode data gathered from Spektrum DX8
		// 2 low pulses: DSM2 1024/22ms (this works with Doug Weibel's PPM Encoder firmware)
		// 3 low pulses: no result
		// 4 low pulses: DSM2 2048/11ms
		// 5 low pulses: no result
		// 6 low pulses: DSMX 22ms
		// 7 low pulses: no result
		// 8 low pulses: DSMX 11ms

		int numPulses = 2;
		for (int i = 0; i < numPulses; i++)
		{
			//PORTD &= ~(1 << PORTD5);
			rxport->PIO_CODR = rxbitmask; // low
			delayMicroseconds(116);
			//PORTD |= (1 << PORTD5);
			rxport->PIO_SODR = rxbitmask; // high
			delayMicroseconds(116);
		}
	}
	else
	{
		Serial.println("Timeout.");
	}

	//DDRD &= ~(1 << DDD5); // Rx as input
	//PORTD &= ~(1 << PORTD5); // Low
	pinMode(GPIO_USART1_RX_SPEKTRUM, INPUT);
}


//
///**
//* Handle DSM satellite receiver bind mode handler
//*
//* @param[in] cmd commands - dsm_bind_power_down, dsm_bind_power_up, dsm_bind_set_rx_out, dsm_bind_send_pulses, dsm_bind_reinit_uart
//* @param[in] pulses Number of pulses for dsm_bind_send_pulses command
//*/
//void
//dsm_bind(uint16_t cmd, int pulses)
//{
//	if (dsm_fd < 0) {
//		return;
//	}
//
//	switch (cmd) {
//
//	case DSM_CMD_BIND_POWER_DOWN:
//
//		/*power down DSM satellite*/
//		POWER_SPEKTRUM(0);
//		break;
//
//	case DSM_CMD_BIND_POWER_UP:
//
//		/*power up DSM satellite*/
//		POWER_SPEKTRUM(1);
//		dsm_guess_format(true);
//		break;
//
//	case DSM_CMD_BIND_SET_RX_OUT:
//
//		/*Set UART RX pin to active output mode*/
//		SPEKTRUM_RX_AS_GPIO();
//		break;
//
//	case DSM_CMD_BIND_SEND_PULSES:
//
//		/*Pulse RX pin a number of times*/
//		for (int i = 0; i < pulses; i++) {
//			dsm_udelay(120);
//			SPEKTRUM_RX_HIGH(false);
//			dsm_udelay(120);
//			SPEKTRUM_RX_HIGH(true);
//		}
//
//		break;
//
//	case DSM_CMD_BIND_REINIT_UART:
//
//		/*Restore USART RX pin to RS232 receive mode*/
//		SPEKTRUM_RX_AS_UART();
//		break;
//
//	}
//}

