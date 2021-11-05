/****
 * Uart.h
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

#include <HardwareSerial.h>
#include <SimpleTimer.h>
#include "common.h"

namespace MHZ19
{
/**
 * @brief Available commands
 * 
 * Support differs by device variant
 */
enum Command {
	CMD_GasConcentration = 0x86,	///< -, B, C
	CMD_CalibrateZeroPoint = 0x87,  ///< -, B
	CMD_CalibrateSpanPoint = 0x88,  ///< -, B
	CMD_SelfCalbrationOnOff = 0x79, ///< B, C
	CMD_SetDetectionRange = 0x99,   ///< B
};

struct Request {
	uint8_t start;
	uint8_t sensor;
	uint8_t command;
	uint8_t data[5];
	uint8_t checksum;
};

struct Response {
	uint8_t start;
	uint8_t command;
	uint8_t data[6];
	uint8_t checksum;
};

enum class Error {
	success,
	incompleteResponse,
	invalidResponse,
	timeout,
};

struct Measurement {
	Error error;
	uint8_t status;
	uint16_t co2_ppm;
	int16_t temperature;
};

using MeasurementCallback = Delegate<void(Measurement& m)>;

/**
 * @brief Access MHZ19 sensor via serial port
 */
class Uart
{
public:
	/**
	 * @brief Use device in UART mode
	 * @param serial Port and pins must be pre-configured
	 */
	Uart(HardwareSerial& serial) : serial(serial)
	{
	}

	/**
	 * @brief Change sensor number field
	 *
	 * Original datasheet identifies first byte as sensor number,
	 * others as 'reserved'.
	 * Provided if this value changes in future hardware versions.
	 */
	void setSensorNumber(uint8_t num)
	{
		sensorNumber = num;
	}

	/**
	 * @brief Get currently configured sensor number
	 */
	uint8_t getSensorNumber() const
	{
		return sensorNumber;
	}

	/**
	 * @brief Enable/disable zero-point auto-calibration feature. On by default.
	 *
	 * Calibrates 400ppm zero point reference automatically at power-on and every 24 hours.
	 * Suitable for home/office environment only. Refer to datasheet for details.
	 */
	void setAutoCalibration(bool enable);

	/**
	 * @brief Calibrate zero point manually
	 *
	 * Sensor must be in stable 400ppm CO2 environment.
	 * Refer to datasheet for detailed procedure.
	 */
	void calibrateZero();

	/**
	 * @brief Calibrate span point
	 *
	 * Typical calibration value is 2000ppm, but must be above 1000ppm.
	 * Refer to datasheet for detailed procedure.
	 */
	void calibrateSpan(uint16_t ppm);

	/**
	 * @brief MHZ-19B has configurable detection range
	 */
	void setDetectionRange(DetectionRange range);

	/**
	 * @brief Read measurement from device
	 */
	bool getMeasurement(MeasurementCallback callback);

protected:
	void sendRequest(Request& request);

private:
	void onData();
	void processResponse(Response& response);
	void returnMeasurement(Measurement& m);

	HardwareSerial& serial;
	uint8_t sensorNumber{0x01}; ///< Factory default
	SimpleTimer timer;
	MeasurementCallback callback;
};

} // namespace MHZ19

String toString(MHZ19::Error error);
