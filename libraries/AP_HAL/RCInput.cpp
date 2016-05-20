// 
// 
// 

#include "RCInput.h"
#include <variant.h>
#include <wiring.h>
#include <DSM/dsm.h>

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

void RCInput::_process_dsm_pulse(uint16_t width_s0, uint16_t width_s1)
{
	// convert to bit widths, allowing for up to 1usec error, assuming 115200 bps
	uint16_t bits_s0 = ((width_s0 + 4)*(uint32_t)115200) / 1000000;
	uint16_t bits_s1 = ((width_s1 + 4)*(uint32_t)115200) / 1000000;
	uint8_t bit_ofs, byte_ofs;
	uint16_t nbits;

	if (bits_s0 == 0 || bits_s1 == 0) {
		// invalid data
		goto reset;
	}

	byte_ofs = dsm_state.bit_ofs / 10;
	bit_ofs = dsm_state.bit_ofs % 10;

	if (byte_ofs > 15) {
		// invalid data
		goto reset;
	}

	// pull in the high bits
	nbits = bits_s0;
	if (nbits + bit_ofs > 10) {
		nbits = 10 - bit_ofs;
	}
	dsm_state.bytes[byte_ofs] |= ((1U << nbits) - 1) << bit_ofs;
	dsm_state.bit_ofs += nbits;
	bit_ofs += nbits;

	if (bits_s0 - nbits > 10) {
		if (dsm_state.bit_ofs == 16 * 10) {
			// we have a full frame
			uint8_t bytes[16];
			uint8_t i;
			for (i = 0; i<16; i++) {
				// get raw data
				uint16_t v = dsm_state.bytes[i];

				// check start bit
				if ((v & 1) != 0) {
					goto reset;
				}
				// check stop bits
				if ((v & 0x200) != 0x200) {
					goto reset;
				}
				bytes[i] = ((v >> 1) & 0xFF);
			}
			uint16_t values[8];
			uint16_t num_values = 0;
			if (dsm_decode(AP_HAL::micros64(), bytes, values, &num_values, 8) &&
				num_values >= 5) {
				for (i = 0; i<num_values; i++) {
					_pwm_values[i] = values[i];
				}
				_num_channels = num_values;
				new_rc_input = true;
			}
		}
		memset(&dsm_state, 0, sizeof(dsm_state));
	}

	byte_ofs = dsm_state.bit_ofs / 10;
	bit_ofs = dsm_state.bit_ofs % 10;

	if (bits_s1 + bit_ofs > 10) {
		// invalid data
		goto reset;
	}

	// pull in the low bits
	dsm_state.bit_ofs += bits_s1;
	return;
reset:
	memset(&dsm_state, 0, sizeof(dsm_state));
}