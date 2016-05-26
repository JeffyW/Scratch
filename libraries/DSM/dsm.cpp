#include <Arduino.h>
#include <AP_Common/AP_Common.h>
#include <wiring_digital.h>
#include "dsm.h"

#define DSM2_22ms_BIND_PULSES 3	/* DSM_BIND_START ioctl parameter, pulses required to start dsm2 pairing */
#define DSM2_11ms_BIND_PULSES 3	/* DSM_BIND_START ioctl parameter, pulses required to start dsm2 pairing */
#define DSMX_22ms_BIND_PULSES 7	/* DSM_BIND_START ioctl parameter, pulses required to start dsmx pairing */
#define DSMX_11ms_BIND_PULSES 9	/* DSM_BIND_START ioctl parameter, pulses required to start 8 or more channel dsmx pairing */

#define SPEKTRUM_RX_AS_UART()	pinMode(hal->dsm_receiver_pin, INPUT); hal->dsm_receiver->begin(115200) //stm32_configgpio(GPIO_USART1_RX)
#define SPEKTRUM_RX_AS_GPIO()	pinMode(hal->dsm_receiver_pin, OUTPUT) //stm32_configgpio(GPIO_USART1_RX_SPEKTRUM)
#define SPEKTRUM_RX_HIGH(_s)	digitalWrite(hal->dsm_receiver_pin, (_s)) //stm32_gpiowrite(GPIO_USART1_RX_SPEKTRUM, (_s))
#define POWER_SPEKTRUM(_s)		digitalWrite(hal->dsm_receiver_power_pin, (_s)) //stm32_gpiowrite(GPIO_SPEKTRUM_PWR_EN, (_s))

static uint64_t dsm_last_frame_time;		/**< Timestamp for start of last dsm frame */
static unsigned dsm_channel_shift;			/**< Channel resolution, 0=unknown, 1=10 bit, 2=11 bit */

#ifdef DEBUG
#else
#define debug(fmt, args...)
#define debug(fmt, args...) HAL::debug(fmt, ##args)
#endif

/**
* Attempt to decode a single channel raw channel datum
*
* The DSM* protocol doesn't provide any explicit framing,
* so we detect dsm frame boundaries by the inter-dsm frame delay.
*
* The minimum dsm frame spacing is 11ms; with 16 bytes at 115200bps
* dsm frame transmission time is ~1.4ms.
*
* We expect to only be called when bytes arrive for processing,
* and if an interval of more than 5ms passes between calls,
* the first byte we read will be the first byte of a dsm frame.
*
* In the case where byte(s) are dropped from a dsm frame, this also
* provides a degree of protection. Of course, it would be better
* if we didn't drop bytes...
*
* Upon receiving a full dsm frame we attempt to decode it
*
* @param[in] raw 16 bit raw channel value from dsm frame
* @param[in] shift position of channel number in raw data
* @param[out] channel pointer to returned channel number
* @param[out] value pointer to returned channel value
* @return true=raw value successfully decoded
*/
static bool dsm_decode_channel(uint16_t raw, unsigned shift, unsigned *channel, unsigned *value)
{

	if (raw == 0xffff)
		return false;

	*channel = (raw >> shift) & 0xf;

	uint16_t data_mask = (1 << shift) - 1;
	*value = raw & data_mask;

	//debug("DSM: %d 0x%04x -> %d %d", shift, raw, *channel, *value);

	return true;
}

static uint32_t	cs10;
static uint32_t	cs11;
static unsigned samples;

static void resetFormat()
{
	cs10 = 0;
	cs11 = 0;
	samples = 0;
	dsm_channel_shift = 0;
}

/**
* Attempt to guess if receiving 10 or 11 bit channel values
*
* @param[in] reset true=reset the 10/11 bit state to unknown
*/
static bool dsm_guess_format(const uint8_t dsm_frame[DSM_FRAME_SIZE])
{
	debug("Frame:	%02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x %02x%02x", dsm_frame[0], dsm_frame[1], dsm_frame[2], dsm_frame[3], dsm_frame[4], dsm_frame[5], dsm_frame[6], dsm_frame[7], dsm_frame[8], dsm_frame[9], dsm_frame[10], dsm_frame[11], dsm_frame[12], dsm_frame[13], dsm_frame[14], dsm_frame[15]);

	/* The first two bytes are some form of a header, but not channel information, thus starting at 1. */
	for (unsigned i = 2; i < DSM_FRAME_SIZE; i += 2) {

		/* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
		const uint8_t *dp = &dsm_frame[i];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		/* if the channel decodes, remember the assigned number */
		if (dsm_decode_channel(raw, 10, &channel, &value) && (channel < 31))
			cs10 |= (1 << channel);

		if (dsm_decode_channel(raw, 11, &channel, &value) && (channel < 31))
			cs11 |= (1 << channel);

		/* XXX if we cared, we could look for the phase bit here to decide 1 vs. 2-dsm_frame format */
	}

	/* wait until we have seen plenty of frames - 5 should normally be enough */
	if (samples++ < 5)
		return true;

	/*
	* Iterate the set of sensible sniffed channel sets and see whether
	* decoding in 10 or 11-bit mode has yielded anything we recognize.
	*
	* XXX Note that due to what seem to be bugs in the DSM2 high-resolution
	*     stream, we may want to sniff for longer in some cases when we think we
	*     are talking to a DSM2 receiver in high-resolution mode (so that we can
	*     reject it, ideally).
	*     See e.g. http://git.openpilot.org/cru/OPReview-116 for a discussion
	*     of this issue.
	*/
	static uint32_t masks[] = {
		0x3f,	/* 6 channels (DX6) */
		0x7f,	/* 7 channels (DX7) */
		0xff,	/* 8 channels (DX8) */
		0x1ff,	/* 9 channels (DX9, etc.) */
		0x3ff,	/* 10 channels (DX10) */
		0x1fff,	/* 13 channels (DX10t) */
		0x3fff	/* 18 channels (DX10) */
	};
	unsigned votes10 = 0;
	unsigned votes11 = 0;

	for (unsigned i = 0; i < ARRAY_SIZE(masks); i++) {

		if (cs10 == masks[i])
			votes10++;

		if (cs11 == masks[i])
			votes11++;
	}

	if ((votes11 == 1) && (votes10 == 0)) {
		dsm_channel_shift = 11;
		debug("DSM: 11-bit format");
		return true;
	}

	if ((votes10 == 1) && (votes11 == 0)) {
		dsm_channel_shift = 10;
		debug("DSM: 10-bit format");
		return true;
	}

	/* call ourselves to reset our state ... we have to try again */
	debug("DSM: format detect fail, 10: 0x%08x %d 11: 0x%08x %d", cs10, votes10, cs11, votes11);
	resetFormat();
	return false;
}

/**
* Decode the entire dsm frame (all contained channels)
*
*/
bool DSM::dsm_decode(const uint8_t dsm_frame[DSM_FRAME_SIZE], uint16_t *values, uint16_t *num_values, uint16_t max_values)
{
	uint64_t frame_time = HAL::micros64();

	/*
	* If we have lost signal for at least a second, reset the
	* format guessing heuristic.
	*/
	if (((frame_time - dsm_last_frame_time) > 1000000) && (dsm_channel_shift != 0))
		resetFormat();

	/* we have received something we think is a dsm_frame */
	dsm_last_frame_time = frame_time;

	/* if we don't know the dsm_frame format, update the guessing state machine */
	if (dsm_channel_shift == 0)
	{
		if (!dsm_guess_format(dsm_frame))
		{
			// Failed to guess format, clear the buffer and start over.
			// This is overkill, but the only way I can see to clear the buffer.
			SPEKTRUM_RX_AS_UART();
			return false;
		}
		return false;
	}


	/*
	* The encoding of the first two bytes is uncertain, so we're
	* going to ignore them for now.
	*
	* Each channel is a 16-bit unsigned value containing either a 10-
	* or 11-bit channel value and a 4-bit channel number, shifted
	* either 10 or 11 bits. The MSB may also be set to indicate the
	* second dsm_frame in variants of the protocol where more than
	* seven channels are being transmitted.
	*/

	for (unsigned i = 2; i < DSM_FRAME_SIZE; i += 2) {

		/* scan the channels in the current dsm_frame in both 10- and 11-bit mode */
		const uint8_t *dp = &dsm_frame[i];
		uint16_t raw = (dp[0] << 8) | dp[1];
		unsigned channel, value;

		if (!dsm_decode_channel(raw, dsm_channel_shift, &channel, &value))
			continue;

		/* ignore channels out of range */
		if (channel >= max_values)
			continue;

		/* update the decoded channel count */
		if (channel >= *num_values)
			*num_values = channel + 1;

		/* convert 0-1024 / 0-2048 values to 1000-2000 ppm encoding. */
		if (dsm_channel_shift == 10)
			value *= 2;

		/*
		* Spektrum scaling is special. There are these basic considerations
		*
		*   * Midpoint is 1520 us
		*   * 100% travel channels are +- 400 us
		*
		* We obey the original Spektrum scaling (so a default setup will scale from
		* 1100 - 1900 us), but we do not obey the weird 1520 us center point
		* and instead (correctly) center the center around 1500 us. This is in order
		* to get something useful without requiring the user to calibrate on a digital
		* link for no reason.
		*/

		/* scaled integer for decent accuracy while staying efficient */
		value = ((((int)value - 1024) * 1000) / 1700) + 1500;

		/*
		* Store the decoded channel into the R/C input buffer, taking into
		* account the different ideas about channel assignement that we have.
		*
		* Specifically, the first four channels in rc_channel_data are roll, pitch, thrust, yaw,
		* but the first four channels from the DSM receiver are thrust, roll, pitch, yaw.
		*/
		switch (channel) {
		case 0:
			channel = 2;
			break;

		case 1:
			channel = 0;
			break;

		case 2:
			channel = 1;

		default:
			break;
		}

		values[channel] = value;
	}

	/*
	* Spektrum likes to send junk in higher channel numbers to fill
	* their packets. We don't know about a 13 channel model in their TX
	* lines, so if we get a channel count of 13, we'll return 12 (the last
	* data index that is stable).
	*/
	if (*num_values == 13)
		*num_values = 12;

#if 0
	if (dsm_channel_shift == 11) {
		/* Set the 11-bit data indicator */
		*num_values |= 0x8000;
	}
#endif

	/*
	* XXX Note that we may be in failsafe here; we need to work out how to detect that.
	*/
	return true;
}

void DSM::init(bool shouldBind)
{
	pinMode(hal->dsm_receiver_power_pin, 0x1);

	if (shouldBind)
	{
		bind(DSMX_11ms_BIND_PULSES);
	}
	else
	{
		// Need to find a way to detect that we're in binding mode, then recycle the RX.
		// But if we're not in binding mode, I don't want to lose the cycles.
		//if (false)
		//{
		//	POWER_SPEKTRUM(/*LOW*/ 0);
		//	HAL::usleep(50000);
		//}
	}

	SPEKTRUM_RX_AS_UART();
	POWER_SPEKTRUM(/*HIGH*/ 1);
}

void DSM::bind(uint8_t pulses)
{
	POWER_SPEKTRUM(/*LOW*/ 0);
	HAL::usleep(500000);
	/*Set UART RX pin to active output mode*/
	SPEKTRUM_RX_AS_GPIO();

	/*power up DSM satellite*/
	POWER_SPEKTRUM(/*HIGH*/ 1);
	resetFormat();
	HAL::usleep(72000);

	/*Pulse RX pin a number of times*/
	for (int i = 0; i < pulses; i++) {
		HAL::udelay(120);
		SPEKTRUM_RX_HIGH(/*LOW*/ 0);
		HAL::udelay(120);
		SPEKTRUM_RX_HIGH(/*HIGH*/ 1);
	}
}
