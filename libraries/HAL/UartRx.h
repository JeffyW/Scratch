#ifndef _HAL_UartRx
#define _HAL_UartRx

#include <stdint.h>
#include "DigitalPin.h"

namespace HAL
{
	class UartRx : public DigitalPin
	{
	public:
		virtual ~UartRx() {}

		virtual uint32_t Available() = 0;
		virtual uint32_t Read() = 0;
		virtual void Begin(uint32_t baud) = 0;
		virtual void Clear() = 0;
	};
}

#endif // !_HAL_UartRx
