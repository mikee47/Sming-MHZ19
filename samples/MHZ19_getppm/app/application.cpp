/*----------------------------------------------------------
    MH-Z19 CO2 sensor  SAMPLE
  ----------------------------------------------------------*/
#include <SmingCore.h>
#include <MHZ19.h>
#include <MHZ19/SampleConfig.h>

namespace
{
SimpleTimer timer;

void takeMeasurement()
{
	Serial.println("Take Measurement");

	mhz19.getMeasurement([](MHZ19::Measurement& m) {
		Serial.print("Measurement result: ");
		Serial.println(toString(m.error));
		if(m.error != MHZ19::Error::success) {
			return;
		}

		Serial.print("co2 ");
		Serial.print(m.co2_ppm);
		Serial.print(", temp ");
		Serial.print(m.temperature);
		Serial.print(", status ");
		Serial.println(m.status);
	});

	/* This method of PWM reading is not recommended */
	// int co2ppm = MHZ19::pwmRead(PWM_PIN);
	// Serial.print("co2 via PWM: ");
	// Serial.println(co2ppm);
}

} // namespace

void init()
{
	initHardware();

	mhz19.setAutoCalibration(false);
	timer.initializeMs<2000>(takeMeasurement).start();

	pwmReader.setCallback([](uint16_t ppm) {
		Serial.print(_F("PWM reader says "));
		Serial.print(ppm);
		Serial.println(_F("ppm"));
	});

	Serial.println("Note: Sensor requires 3 minutes to warm up.");
}
