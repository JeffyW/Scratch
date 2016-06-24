#ifndef _HAL
#define _HAL

#define VERBOSE 0x01
#define INFORMATIONAL 0x02
#define WARNING 0x04
#define ERROR 0x08
#define CRITICAL 0x10

#define debug_GPS 0x01
#define debug_IMU 0x02
#define debug_DSM 0x04

namespace HAL
{
	class HAL
	{
	public:
		HAL(UARTClass* _console)
			:
			console(_console)
		{
			//_console->begin(115200);
		}

		Print* console;

		//void debug();

		void init()
		{
			((UARTClass*)console)->begin(250000);
		}
	};

	void udelay(uint32_t usec);
	void usleep(uint32_t usec);

	uint32_t micros();
	uint32_t millis();
	uint64_t micros64();
	uint64_t millis64();

	static uint16_t SourceMask = debug_IMU;
	static uint16_t LevelMask = 0xff;

	void debug(const char *fmt, ...);
	void debug(uint16_t source, uint16_t level, const char *fmt, ...);
}

#endif