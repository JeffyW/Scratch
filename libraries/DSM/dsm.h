/*
  declaration of dsm_decode from dsm.cpp
 */
#include <stdint.h>

#define DSM2_BIND_PULSES 3	/* DSM_BIND_START ioctl parameter, pulses required to start dsm2 pairing */
#define DSMX_BIND_PULSES 7	/* DSM_BIND_START ioctl parameter, pulses required to start dsmx pairing */
#define DSMX8_BIND_PULSES 9 	/* DSM_BIND_START ioctl parameter, pulses required to start 8 or more channel dsmx pairing */
#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes*/

enum DSM_CMD {							/* DSM bind states */
	DSM_CMD_BIND_POWER_DOWN = 0,
	DSM_CMD_BIND_POWER_UP,
	DSM_CMD_BIND_SET_RX_OUT,
	DSM_CMD_BIND_SEND_PULSES,
	DSM_CMD_BIND_REINIT_UART
};

bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[16],
                uint16_t *values, 
                uint16_t *num_values, 
                uint16_t max_values);

void
dsm_guess_format(bool reset, const uint8_t dsm_frame[DSM_FRAME_SIZE]);