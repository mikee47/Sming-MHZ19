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

	pwmReader.begin(PWM_PIN);
}
