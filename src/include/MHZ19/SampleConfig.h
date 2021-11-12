/*
 * Common code for initialising serial ports
 */

#define PWM_PIN 14

#define SERIAL_TX_PIN 2
#define MHZ19_TX_PIN 15
#define MHZ19_RX_PIN 13

namespace
{
HardwareSerial serial0(0);
MHZ19::Uart mhz19(serial0);
MHZ19::PwmReader pwmReader;
} // namespace

void initHardware()
{
	serial0.begin();
	serial0.swap();

	Serial.setPort(1);
	Serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, SERIAL_TX_ONLY, SERIAL_TX_PIN);
	Serial.systemDebugOutput(true);

	/**
	 * @brief MH-Z19 devices work best at 2000 PPM range.
	 * Above this the accuracy drops off.
	 */
	auto range = MHZ19::DetectionRange::PPM_2000;

	mhz19.setDetectionRange(range);
	pwmReader.begin(PWM_PIN, range);
}
