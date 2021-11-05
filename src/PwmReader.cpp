/**
 * PwmReader.cpp
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

#include "include/MHZ19/PwmReader.h"
#include <Digital.h>
#include <Platform/Clocks.h>
#include <muldiv.h>

// Ignore measurements with cycle times outside (CYCLE_MS +/- TOLERANCE)
#define PWM_CYCLE_MS 1004
#define PWM_CYCLE_TOLERANCE 250

#ifdef ARCH_HOST
#include <DigitalHooks.h>
#endif

namespace MHZ19
{
// Required by ISR
PwmReader* PwmReader::self;

namespace
{
#ifdef ARCH_HOST
class Hooks : public DigitalHooks
{
public:
	/*
	 * From datasheet:
	 *
	 *   ppm = 2000 * (Th - 2) / (Th + Tl - 4)
	 *
	 * Cycle time is 1004ms so Tl = 1004 - Th:
	 *
	 *   ppm = 2000 * (Th - 2) / (Th + 1004 - Th - 4)
	 *       = 2000 * (Th - 2) / 1000
	 *       = 2 * (Th - 2)
	 * 
	 * Gives:
	 *
	 * 		Th = (ppm / 2) + 2
	 */
	unsigned long pulseIn(uint16_t pin, uint8_t state, unsigned long timeout) override
	{
		auto ppm = 400 + os_random() % (2000 - 400);
		// Result in microseconds
		return 1000 * (ppm / 2) + 2;
	}
};

Hooks hooks;

#endif

uint16_t calculatePpm(PwmReader::Pulse pulse, DetectionRange range)
{
	auto cycleTime = pulse.low + pulse.high;
	if(pulse.low <= 2 || pulse.high <= 2 || cycleTime < (PWM_CYCLE_MS - PWM_CYCLE_TOLERANCE) ||
	   cycleTime > (PWM_CYCLE_MS + PWM_CYCLE_TOLERANCE)) {
		return 0;
	}
	pulse.high -= 2;
	pulse.low -= 2;

	return muldiv(pulse.high, uint16_t(range), uint16_t(pulse.high + pulse.low));
}

} // namespace

void PwmReader::begin(uint8_t pin, DetectionRange range)
{
	this->pin = pin;
	this->range = range;
	reading.value = 0;
	self = this;
	attachInterrupt(pin, staticInterruptHandler, CHANGE);
	pinMode(pin, INPUT_PULLUP);
}

void PwmReader::end()
{
	detachInterrupt(pin);
	reading.value = 0;
}

void IRAM_ATTR PwmReader::interruptHandler()
{
	auto ticks = PolledTimerClock::ticks();
	auto ms = PolledTimerClock::ticksToTime<NanoTime::Milliseconds>(ticks - isrTicks);
	isrTicks = ticks;
	if(digitalRead(pin)) {
		if(isrPulse.high != 0) {
			reading.value = isrPulse.value;
		}
		isrPulse.low = ms;
	} else {
		isrPulse.high = ms;
	}
}

uint16_t PwmReader::getMeasurement() const
{
	Pulse r{.value = reading.value};
	auto ppm = calculatePpm(r, range);
	debug_d("[MHZ19] PWM reading (%u, %u) -> %u ppm", r.high, r.low, ppm);
	return ppm;
}

unsigned pwmRead(uint8_t pwmPin, DetectionRange range)
{
#ifdef ARCH_HOST
	setDigitalHooks(&hooks);
#endif

	PwmReader::Pulse pulse;
	pulse.high = pulseIn(pwmPin, HIGH, 1000 * PWM_CYCLE_MS * 2) / 1000;
	pulse.low = 1004 - pulse.high;
	return calculatePpm(pulse, range);
}

} // namespace MHZ19
