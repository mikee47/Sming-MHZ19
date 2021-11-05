/*----------------------------------------------------------
    MH-Z19 CO2 sensor  SAMPLE
  ----------------------------------------------------------*/
#include <SmingCore.h>
#include <MHZ19.h>
#include <MHZ19/SampleConfig.h>

#ifdef ARCH_HOST
// Speed things up for testing
#define WAITING_MINUTES 5
#define TIMER_INTERVAL 100
#else
#define WAITING_MINUTES 30
#define TIMER_INTERVAL 1000
#endif

namespace
{
enum class State {
	start,
	readPwm,
	check,
	done,
};

SimpleTimer timer;
unsigned tickCount;
uint32_t averageSum;
State state;

void calibrateReading(uint16_t ppm)
{
	averageSum += ppm;
	Serial.print("\r#");
	Serial.print(tickCount / 60);
	Serial.print(": co2 = ");
	Serial.print(ppm);
	Serial.println(" ppm");

	if(ppm == 0) {
		Serial.println(_F("WARNING: Looks like PWM input isn't connected."));
	}

	if(tickCount < WAITING_MINUTES * 60) {
		return;
	}

	tickCount = 0;
	auto avg = averageSum / WAITING_MINUTES;
	Serial.print(_F("CO2 Average = "));
	Serial.println(avg);
	if(avg < 1000) {
		Serial.println(_F("Cannot calibrate span point as CO2 average below 1000ppm"));
		// Restart calibration
		state = State::start;
		return;
	}

	Serial.println();
	Serial.println(_F("Performing check readings..."));
	mhz19.calibrateSpan(avg);
	state = State::check;
}

void calibrate()
{
	++tickCount;

	switch(state) {
	case State::start:
		if(tickCount == 2) {
			Serial.println(_F("Reading PWM input for " STR(WAITING_MINUTES) " minutes"));
			tickCount = 0;
			averageSum = 0;
			state = State::readPwm;
		}
		break;

	case State::readPwm:
		// Read PWM input once per minute
		if(tickCount % 60 == 0) {
			calibrateReading(pwmReader.getMeasurement());
		} else {
			Serial.printf("\r%2u", 60 - (tickCount % 60));
		}
		break;

	case State::check:
		if(tickCount % 10 == 0) {
			mhz19.getMeasurement([](MHZ19::Measurement& m) {
				if(m.error == MHZ19::Error::success) {
					Serial.print("\rco2: ");
					Serial.print(m.co2_ppm);
					Serial.println("ppm");
				} else {
					Serial.println(toString(m.error));
				}
				if(tickCount >= 100) {
					state = State::done;
				}
			});
		} else {
			Serial.print('.');
		}
		break;

	case State::done:
		Serial.println(_F("Done."));
		timer.stop();
		break;
	}
}

} // namespace

void init()
{
	initHardware();

	Serial.println(_F("Note: Sensor requires 3 minutes to warm up."));

	mhz19.setAutoCalibration(false);
	timer.initializeMs<TIMER_INTERVAL>(calibrate).start();
}
