#include <Arduino.h>
#include <stdarg.h>
#include "HAL.h"

namespace HAL
{
	/* Yield to the scheduler if possible. */
	void usleep(uint32_t usec)
	{
		::delayMicroseconds(usec);
	}

	/* Do not yield. */
	void udelay(uint32_t usec)
	{
		::delayMicroseconds(usec);
	}

	uint32_t micros()
	{
		return ::micros();
	}

	uint32_t millis()
	{
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

	void debug(uint16_t source, uint16_t level, const char *fmt, ...)
	{
		if ((source & SourceMask) && (level & LevelMask))
		{
			char buf[128]; // resulting string limited to 128 chars
			va_list args;
			va_start(args, fmt);
			vsnprintf(buf, 128, fmt, args);
			va_end(args);
			Serial.println(buf);
		}
	}

	void debug(const char *fmt, ...)
	{
		char buf[128]; // resulting string limited to 128 chars
		va_list args;
		va_start(args, fmt);
		vsnprintf(buf, 128, fmt, args);
		va_end(args);
		Serial.println(buf);
	}
}
