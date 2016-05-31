#ifndef _HAL_DigitalPin
#define _HAL_DigitalPin

#include <stdint.h>

namespace HAL
{
	class DigitalPin
	{
	public:
		virtual ~DigitalPin() {}

		virtual void SetMode(uint32_t mode) = 0;
		virtual void Write(uint32_t value) = 0;
	};
}

#endif // !_HAL_DigitalPin