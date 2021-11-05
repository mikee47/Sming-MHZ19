/**
 * MHZ19.cpp
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

#include "MHZ19.h"
#include <Digital.h>
#include <Platform/Clocks.h>
#include <muldiv.h>
#include <numeric>

#define START 0xff
#define UART_TIMEOUT_MS 500

// Ignore measurements with cycle times outside (CYCLE_MS +/- TOLERANCE)
#define PWM_CYCLE_MS 1004
#define PWM_CYCLE_TOLERANCE 250

#ifdef ARCH_HOST
#include <DigitalHooks.h>
#endif

String toString(MHZ19::Error error)
{
	switch(error) {
	case MHZ19::Error::success:
		return F("Success");
	case MHZ19::Error::incompleteResponse:
		return F("Incomplete Response");
	case MHZ19::Error::invalidResponse:
		return F("Invalid Response");
	case MHZ19::Error::timeout:
		return F("Timeout");
	default:
		assert(false);
		return nullptr;
	}
}

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

template <typename T> uint8_t calculateChecksum(const T& packet)
{
	auto data = &packet.start;
	return -std::accumulate(&data[1], &data[7], 0);
}

uint16_t calculatePpm(PwmReader::Pulse pulse, DetectionRange range)
{
	auto cycleTime = pulse.low + pulse.high;
	if(pulse.low <= 2 || pulse.high <= 2 || cycleTime < (PWM_CYCLE_MS - PWM_CYCLE_TOLERANCE) ||
	   cycleTime > (PWM_CYCLE_MS + PWM_CYCLE_TOLERANCE)) {
		return 0;
	}
	pulse.high -= 2;
	pulse.low -= 2;

	return muldiv(pulse.high, uint16_t(range), pulse.high + pulse.low);
}

} // namespace

void Uart::setAutoCalibration(bool enable)
{
	Request req{
		.command = CMD_SelfCalbrationOnOff,
		.data = {uint8_t(enable ? 0xa0 : 0x00)},
	};
	sendRequest(req);
}

void Uart::calibrateZero()
{
	Request req{
		.command = CMD_CalibrateZeroPoint,
	};
	sendRequest(req);
}

void Uart::calibrateSpan(uint16_t ppm)
{
	if(ppm < 1000) {
		return;
	}

	Request req{
		.command = CMD_CalibrateSpanPoint,
		.data = {uint8_t(ppm >> 8), uint8_t(ppm)},
	};
	sendRequest(req);
}

void Uart::setDetectionRange(DetectionRange range)
{
	auto r = uint32_t(range);
	Request req{
		.command = CMD_SetDetectionRange,
		.data = {0, uint8_t(r >> 24), uint8_t(r >> 16), uint8_t(r >> 8), uint8_t(r)},
	};
	sendRequest(req);
}

void Uart::sendRequest(Request& request)
{
	request.start = START;
	request.sensor = sensorNumber;
	request.checksum = calculateChecksum(request);
	debug_hex(DBG, "> MHZ19", &request, sizeof(request));
	serial.write(&request.start, sizeof(request));
}

void Uart::onData()
{
	Response response{};
	auto len = serial.readBytes(reinterpret_cast<char*>(&response), sizeof(response));
	debug_hex(DBG, "< MHZ19", &response, len);
	if(len != sizeof(response)) {
		debug_e("[MHZ19] Response too short %u", len);
		Measurement m{Error::incompleteResponse};
		returnMeasurement(m);
	} else {
		processResponse(response);
	}
}

void Uart::processResponse(Response& response)
{
	Measurement m{Error::invalidResponse};
	auto checksum = calculateChecksum(response);
	if(response.start != START) {
		debug_w("[MHZ19] Start invalid 0x%02x", response.start);
	} else if(response.command != CMD_GasConcentration) {
		debug_w("[MHZ19] Response command invalid 0x%02u", response.command);
	} else if(checksum != response.checksum) {
		debug_w("[MHZ19] Checksum failed");
	} else {
		m.error = Error::success;
		m.co2_ppm = (response.data[0] << 8) | response.data[1];
		m.temperature = int(response.data[2]) - 40;
		m.status = response.data[3];
	}
	returnMeasurement(m);
}

void Uart::returnMeasurement(Measurement& m)
{
	timer.stop();
	serial.onDataReceived(nullptr);
	debug_d("[MHZ19] Returning %s", toString(m.error).c_str());
	auto cb = callback;
	callback = nullptr;
	assert(cb);
	cb(m);
}

bool Uart::getMeasurement(MeasurementCallback callback)
{
	if(this->callback) {
		debug_e("[MHZ19] Measurement already in progress");
		return false;
	}

	this->callback = callback;
	serial.clear();
	serial.onDataReceived([this](Stream&, char, unsigned short) { onData(); });
	timer.initializeMs<UART_TIMEOUT_MS>(
		[](void* param) {
			auto self = static_cast<Uart*>(param);
#ifdef ARCH_HOST
			// Create dummy response
			auto ppm = 400 + os_random() % (2000 - 400);
			Response response{
				.start = START,
				.command = CMD_GasConcentration,
				.data = {uint8_t(ppm >> 8), uint8_t(ppm), 25 + 40},
				.checksum = calculateChecksum(response),
			};
			debug_hex(DBG, "< MHZ19", &response, sizeof(response));
			self->processResponse(response);
#else
			Measurement m{.error = Error::timeout};
			self->returnMeasurement(m);
#endif
		},
		this);
	timer.startOnce();

	Request req{
		.command = CMD_GasConcentration,
	};
	sendRequest(req);
	return true;
}

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
