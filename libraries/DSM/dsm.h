/*
declaration of dsm_decode from dsm.cpp
*/
#define DSM_FRAME_SIZE		16		/**< DSM frame size in bytes*/

bool dsm_decode(uint64_t frame_time, const uint8_t dsm_frame[DSM_FRAME_SIZE],
	uint16_t *values,
	uint16_t *num_values,
	uint16_t max_values);


void
dsm_guess_format(bool reset, const uint8_t dsm_frame[DSM_FRAME_SIZE]);