#ifndef _HAL
#define _HAL

namespace HAL
{
	class HAL
	{
	public:
		HAL(UARTClass* _console,
			UARTClass* _dsm_receiver,
			uint32_t _dsm_receiver_pin,
			uint32_t _dsm_receiver_power_pin)
			:
			console(_console),
			dsm_receiver(_dsm_receiver),
			dsm_receiver_pin(_dsm_receiver_pin),
			dsm_receiver_power_pin(_dsm_receiver_power_pin)
		{
			//_console->begin(115200);
		}

		Print* console;
		UARTClass* dsm_receiver;
		uint32_t dsm_receiver_pin;
		uint32_t dsm_receiver_power_pin;

		//void debug();

		void init()
		{
			((UARTClass*)console)->begin(115200);
		}
	};

	void udelay(uint32_t usec);
	void usleep(uint32_t usec);

	uint32_t micros();
	uint32_t millis();
	uint64_t micros64();
	uint64_t millis64();

	void debug(const char *fmt, ...);
}

#endif