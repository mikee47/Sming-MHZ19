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
	calibrate1,
	check,
	done,
};

SimpleTimer timer;
unsigned tickCount;
State state;

void calibrate()
{
	++tickCount;

	switch(state) {
	case State::start:
		if(tickCount == 2) {
			Serial.println(_F("Reading PWM input for " STR(WAITING_MINUTES) " minutes"));
			tickCount = 0;
			state = State::readPwm;
		}
		break;

	case State::readPwm:
		// Read PWM input once per minute
		if(tickCount % 60 == 0) {
			auto ppm = pwmReader.getMeasurement();
			Serial.print("\r#");
			Serial.print(tickCount / 60);
			Serial.print(": co2 = ");
			Serial.print(ppm);
			Serial.println(" ppm");

			if(ppm == 0) {
				Serial.println(_F("WARNING: Looks like PWM input isn't connected."));
			}

			if(tickCount >= WAITING_MINUTES * 60) {
				Serial.println(_F("\rFirst zero-calibration"));
				mhz19.calibrateZero();
				tickCount = 0;
				state = State::calibrate1;
			}
		} else {
			Serial.printf("\r%2u", 60 - (tickCount % 60));
		}
		break;

	case State::calibrate1:
		if(tickCount >= 60) {
			Serial.println(_F("\rSecond zero-calibration"));
			mhz19.calibrateZero();
			tickCount = 0;
			state = State::check;
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
