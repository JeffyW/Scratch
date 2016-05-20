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

#define pin 48

void setup()
{
	Serial.begin(115200);

	//pinMode(pin, OUTPUT);
	//digitalWrite(pin, LOW);
	Serial3.begin(115200); //Uses Serial3 for input as default
	//while (Serial3.available() > 0)
	//{
	//	debug("Flushing");
	//	Serial3.read();
	//}
	//delay(1 * 1000);
	//digitalWrite(pin, HIGH);
}


void loop()
{
	static uint16_t numChannels;
	static uint16_t values[16];

	while (Serial3.available() >= 16)
	{
		uint8_t i;
		uint8_t dsm_frame[16];
		for (i = 0; i < 16; i++)
		{
			dsm_frame[i] = Serial3.read();
		}

		//debug("DSM dsm_frame %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x",
		//	dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7],
		//	dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);

		//for (i = 1; i < 8; i++)
		//{
		//	uint16_t raw = (dsm_frame[2 * i] << 8) | dsm_frame[2 * i + 1];

		//	uint16_t channel = raw >> 11;

		//	uint16_t value = raw << 5;
		//	value = value >> 5;

		//	if (channel < 9)
		//	{
		//		values[channel] = value - 342;
		//	}
		//	else
		//	{
		//		//error, ignore ;)
		//	}
		//}

		for (i = 1; i < 8; i++)
		{
			unsigned channel;
			unsigned value;
			uint16_t raw = (dsm_frame[2 * i] << 8) | dsm_frame[2 * i + 1];
			dsm_decode_channel(raw, 11, &channel, &value);
			//debug("DSM: 0x%04x -> %d %d", raw, channel, value);
			values[channel] = value;
		}

		//uint16_t num_values = 0;
		//if (!dsm_decode(AP_HAL::micros64(), dsm_frame, values, &num_values, numChannels)) {
		//	debug("WTF");
		//}

		debug("%u	%u	%u	%u	%u	%u	%u	%u	%u", values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7], values[8]);
	}
}
