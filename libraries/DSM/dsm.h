/*
declaration of dsm_decode from dsm.cpp
*/
#include <Print.h>
#include <USARTClass.h>

#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes*/

bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[DSM_FRAME_SIZE],
	uint16_t *values,
	uint16_t *num_values,
	uint16_t max_values);

enum DSM_CMD {							/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES,
	DSM_CMD_BIND_REINIT_UART
};

namespace HAL
{
	class HAL
	{
	public:
		HAL(Print* _console,
			USARTClass* _dsm_receiver,
			uint32_t _dsm_receiver_pin,
			uint32_t _dsm_receiver_power_pin)
			:
			console(_console),
			dsm_receiver(_dsm_receiver),
			dsm_receiver_pin(_dsm_receiver_pin),
			dsm_receiver_power_pin(_dsm_receiver_power_pin)
		{}

		Print* console;
		USARTClass* dsm_receiver;
		uint32_t dsm_receiver_pin;
		uint32_t dsm_receiver_power_pin;

		//void debug();
	};
}

class DSM
{
public:
	DSM(HAL::HAL* _hal)
		: hal(_hal)
	{}

	void bind(uint8_t pulses);

private:
	HAL::HAL* hal;
	void bind(uint16_t cmd, int pulses);
};
