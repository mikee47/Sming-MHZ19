/****
 * Uart.cpp
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

#include "include/MHZ19/Uart.h"
#include <Digital.h>
#include <numeric>

#define START 0xff
#define UART_TIMEOUT_MS 500

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
namespace
{
template <typename T> uint8_t calculateChecksum(const T& packet)
{
	auto data = &packet.start;
	return -std::accumulate(&data[1], &data[8], 0);
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

} // namespace MHZ19
