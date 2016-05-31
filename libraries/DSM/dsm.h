#ifndef _DSM
#define _DSM

#include <HAL/DigitalPin.h>
#include <HAL/UartRx.h>

#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes*/

class DSM
{
public:
	DSM(HAL::DigitalPin* receiverPower, HAL::UartRx* receiverRx)
		:
		_receiverPower(receiverPower),
		_receiverRx(receiverRx)
	{}

	void init(bool bind);
	bool dsm_decode(
		const uint8_t dsm_frame[DSM_FRAME_SIZE],
		uint16_t *values,
		uint16_t *num_values,
		uint16_t max_values);

	void loop();

private:
	HAL::DigitalPin* _receiverPower;
	HAL::UartRx* _receiverRx;

	uint32_t cs10; // Bitmask of which channels were decoded using a 10 shift
	uint32_t cs11; // Bitmask of which channels were decoded using an 11 shift
	unsigned samples; // Number of frames sampled for format guessing
	bool binding = false; // Whether we are currenly binding
	uint32_t last_frame_time; // Micros timestamp since the start of the last time any data was received
	unsigned dsm_channel_shift; /**< Channel resolution, 0=unknown, 1=10 bit, 2=11 bit */

	// Initiate the binding process
	void bind(uint8_t pulses);

	bool dsm_guess_format(const uint8_t dsm_frame[DSM_FRAME_SIZE]);

	void resetFormat();
};

#endif