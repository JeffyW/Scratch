#include <HAL_Arduino/UartRx.h>
#include <HAL_Arduino/DigitalPin.h>
#include <HAL/HAL.h>
#include <DSM/dsm.h>

#define BIND_PIN 50

// Serial pin
#define GPIO_USART1_RX_SPEKTRUM		15 
#define GPIO_USART1_RX    (GPIO_INPUT|GPIO_CNF_INFLOAT|GPIO_MODE_INPUT|GPIO_PORTB|GPIO_PIN7)
// Power pin
#define GPIO_SPEKTRUM_PWR_EN		48 
Pio *rxport = digitalPinToPort(GPIO_USART1_RX_SPEKTRUM);
uint32_t rxbitmask = digitalPinToBitMask(GPIO_USART1_RX_SPEKTRUM);
unsigned long timex;

#define debug(fmt, args...) HAL::debug(fmt, ##args)

static HAL_Arduino::UartRx _receiverRx = HAL_Arduino::UartRx(15, &Serial3);
static HAL_Arduino::DigitalPin _receiverPower = HAL_Arduino::DigitalPin(48);

static HAL::HAL hal = HAL::HAL(
	&Serial,
	&Serial3,
	15,//(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN10)
	48);//(GPIO_OUTPUT|GPIO_CNF_OUTPP|GPIO_MODE_50MHz|GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN13)

static DSM dsm = DSM(
	&_receiverPower,
	&_receiverRx);

void setup()
{
	hal.init();
	pinMode(BIND_PIN, INPUT_PULLUP);
	dsm.init(digitalRead(BIND_PIN) == LOW);
}

void loop()
{
	//debug("Main: %llu", HAL::micros64());
	dsm.loop();
}

//
//void SpektrumBind(void)
//{
//	// This code is using Serial as the RX input.
//	// This maps to pin 0, aka PORTD0
//
//	unsigned char connected = 0;
//
//	// Connect the power for the Rx to RX_powerpin this is brought
//	// high to turn on the Rx.
//	delay(5);  // Delay added to work with Orange Receivers
//	digitalWrite(GPIO_SPEKTRUM_PWR_EN, HIGH);
//
//	// Registers for SAM3X series
//	// There are 5 ports available, each is 32bits, mapped to the pins documented in variant.cpp
//	//   PIOx where x is A,B,C,D,E
//	// digitalPinToPort can be used to find which port a pin is mapped to
//	// digitalPinToBitMask can be used to find which bit in a port a pin is mapped to
//	//
//	// There are several registers for each port that read and write
//	// See Component_pio for reference
//	// PIO_PER - Write 1's here to override other peripherals and allow GPIO use for pins
//	// PIO_OER - Write 1's here to set pins as OUTPUT
//	// PIO_ODR - Write 1's here to set pins as INPUT
//
//	// PIO_SODR - Write 1's here to set output pins as HIGH
//	// PIO_CODR - Write 1's here to set output pins as LOW
//	// PIO_PDSR - Read the state of the port
//	// PIO_ODSR - Status register (I think you can read and write here)
//
//	// PIO_IER - Write 1's here to enable interrupts
//	// PIO_IDR - Write 1's here to disable interrups
//
//	// PIO_PUDR - write 1's here to switch OFF internal pull-up for pins (INPUT)
//	// PIO_PUER - write 1's here to switch ON internall pull-up for pins (INPUT_PULLUP)
//	// OTHERS for stuff
//
//	// These are registers for the PORTs
//	// Each bit in the register maps to a port number
//	// Therefore, PORTD B0001 maps to pin 0 on Port D
//	// DDRx		- Data Direction Register.  Sets INPUT/OUTPUT.  Read/Write
//	// PORTx	- Data Register.  Sets state of pin as HIGH/LOW.  Read/Write
//	//    Is that just for OUTPUT or does it also set Pullup/Pulldown?
//	// PINx		- Port Input Register.  Reads current state of pin.  Read
//
//	// Serial / USART registers
//	// UBRR0H - Upper 8 bits of the baud rate value
//	// UBRR0L - Lower 8 bits of the baud rate value
//	//    Note that the U2X0 and clock must be taken account for.
//	// UCSR0C - Character size?
//	//    UMSEL01:0(7,6) - Mode of communication.  00 is Async USART
//	//    UPM01:0(5,4)   - Parity behavior.  00 is disabled
//	//    USBS0(3)       - Bit singlaling - 0 is 1-bit stop, 1 is 2-bit stop
//	//    UCSZ00
//	//    UCSZ01:0(2,1)  - Along with UCSZ02(2) in UCSR0B sets number of data bits.
//	//    UCSZ10
//	//    (0) - Clock polarity.  0 means TX Rising, RX Falling
//	// UCSR0B - interrupts
//	//    RXEN0  4 - Receive
//	//    TXEN0  3 - Transmit
//	//    RXCIE0 7 - Receive interrupt
//
//
//	//UCSR0B &= ~(1 << RXCIE0); // disable Rx interrupt
//	//UCSR0B &= ~(1 << RXEN0); // disable USART1 Rx
//	rxport->PIO_IDR = rxbitmask; // Disable RX interrupt
//	rxport->PIO_PDR = rxbitmask; // Disable GPIO
//
//	//PORTD &= ~(1 << PORTD0); // disable pull-up
//	rxport->PIO_CODR = rxbitmask; // disable pull-up
//
//	while (timex <= 10000) // Wait 10 seconds for spektrum sat connection
//	{
//		timex = millis();
//		if (rxport->PIO_PDSR & rxbitmask)
//			//if (PIND & (1 << PORTD5))
//		{
//			connected = 1;
//			break;
//		}
//	}
//
//	if (connected)
//	{
//		debug("Connected! Binding now!");
//
//		//DDRD |= (1 << DDD5); // Rx as output
//		pinMode(GPIO_USART1_RX_SPEKTRUM, OUTPUT);
//		delay(90); // Delay after Rx startup
//
//		int numPulses = 2;
//		for (int i = 0; i < numPulses; i++)
//		{
//			//PORTD &= ~(1 << PORTD5);
//			rxport->PIO_CODR = rxbitmask; // low
//			delayMicroseconds(116);
//			//PORTD |= (1 << PORTD5);
//			rxport->PIO_SODR = rxbitmask; // high
//			delayMicroseconds(116);
//		}
//	}
//	else
//	{
//		Serial.println("Timeout.");
//	}
//
//	//DDRD &= ~(1 << DDD5); // Rx as input
//	//PORTD &= ~(1 << PORTD5); // Low
//	pinMode(GPIO_USART1_RX_SPEKTRUM, INPUT);
//}
//
