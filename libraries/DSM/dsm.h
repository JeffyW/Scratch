#ifndef _DSM
#define _DSM

#include <Print.h>
#include <USARTClass.h>
#include <HAL/HAL.h>

#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes*/

enum DSM_CMD {							/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES
};

class DSM
{
public:
	DSM(HAL::HAL* _hal)
		: hal(_hal)
	{}

	void init(bool bind);
	bool dsm_decode(
		const uint8_t dsm_frame[DSM_FRAME_SIZE],
		uint16_t *values,
		uint16_t *num_values,
		uint16_t max_values);


private:
	HAL::HAL* hal;

	void bind(uint8_t pulses);
};

#endif