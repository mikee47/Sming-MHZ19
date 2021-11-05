/****
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
 *
 * The ESP8266 lacks any timing capture hardware but as the pulse output
 * from the MHZ19 is very slow it can be decoded using interrupts with high accuracy.
 *
 * Once started, the PwmReader runs continuously and the last value can be obtained
 * by calling `getMeasurement`.
 */
class PwmReader
{
public:
	/**
	 * @brief Used internally to measure a high/low pulse pair
	 */
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

	/**
	 * @brief Start the PWM reader
	 * @param pin GPIO to read PWM signal on
	 * @param range Configured device range
	 *
	 * Runs continuously in background until end() is called.
	 */
	void begin(uint8_t pin, DetectionRange range = DetectionRange::PPM_2000);

	/**
	 * @brief Stop the PWM reader.
	 */
	void end();

	/**
	 * @brief Obtain the most recent measurement.
	 */
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
 * @note This will hang CPU for 1-2 seconds. Use PwmReader instead.
 */
unsigned pwmRead(uint8_t pwmPin, DetectionRange range = DetectionRange::PPM_2000);

} // namespace MHZ19
