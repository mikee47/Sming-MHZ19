/**
 * PwmReader.h
 *
 * Copyright 2021 mikee47 <mike@sillyhouse.net>
 *
 * This file is part of the Sming-MHZ19 Library
 *
 * This library is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, version 3 or later.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.
 * If not, see <https://www.gnu.org/licenses/>.
 *
 ****/

#pragma once

#include "common.h"
#include <cstdint>
#include <esp_attr.h>

namespace MHZ19
{
/**
 * @brief Reads input pulse width asynchronously
 */
class PwmReader
{
public:
	union Pulse {
		struct {
			uint16_t low;
			uint16_t high;
		};
		uint32_t value;
	};

	~PwmReader()
	{
		end();
	}

	void begin(uint8_t pin, DetectionRange range = DetectionRange::PPM_2000);

	void end();

	uint16_t getMeasurement() const;

private:
	static void IRAM_ATTR staticInterruptHandler()
	{
		return self->interruptHandler();
	}

	void interruptHandler();

	static PwmReader* self;
	uint8_t pin;
	uint32_t isrTicks{0};
	Pulse isrPulse{};
	volatile Pulse reading{};
	DetectionRange range;
};

/**
 * @brief Read PWM output from sensor
 * @param pwmPin GPIO to which the sensor is connected
 * @param range Range sensor is configured for
 */
unsigned pwmRead(uint8_t pwmPin, DetectionRange range = DetectionRange::PPM_2000);

} // namespace MHZ19
