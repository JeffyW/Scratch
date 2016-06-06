#include <HAL_Arduino/UartRx.h>
#include <HAL_Arduino/DigitalPin.h>
#include <HAL/HAL.h>
#include <DSM/dsm.h>

static HAL_Arduino::UartRx _receiverRx = HAL_Arduino::UartRx(15, &Serial3);
static HAL_Arduino::DigitalPin _receiverPower = HAL_Arduino::DigitalPin(48);
static HAL_Arduino::DigitalPin _bind = HAL_Arduino::DigitalPin(50);

static HAL::HAL hal = HAL::HAL(&Serial);

static DSM dsm = DSM(
	&_receiverPower,
	&_receiverRx);

void setup()
{
	hal.init();
	_bind.SetMode(INPUT_PULLUP);
	dsm.init(_bind.Read() == LOW);
}

void loop()
{
	// Add some tests to see how long it takes to do a debug.
	// Also test how long it takes to read from the serial in.
	// Consider switching from a full read at a time to read individual bytes on each loop.
	// Also consider that doing so allows us to check time since last byte, allowing better
	// detection of the start of a new frame.

	//debug("Main: %llu", HAL::micros64());
	dsm.loop();
}
