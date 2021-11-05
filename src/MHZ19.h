/*
  MHZ19.h - MH-Z19 CO2 sensor library for ESP-WROOM-02/32(ESP8266/ESP32) or Arduino
  version 1.0
  
  License MIT
*/

#pragma once

#include <HardwareSerial.h>
#include <SimpleTimer.h>

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

enum class DetectionRange {
	PPM_2000 = 2000,
	PPM_5000 = 5000,
	PPM_10000 = 10000,
};

struct Measurement {
	Error error;
	uint8_t status;
	uint16_t co2_ppm;
	int16_t temperature;
};

using MeasurementCallback = Delegate<void(Measurement& m)>;

class Uart
{
public:
	/**
	 * @brief Use device in UART mode
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

String toString(MHZ19::Error error);
