MHZ19 CO2 Sensor
================

Sming library supporting the MH-Z19 and MH-Z19B CO2 sensors.

Essentially a re-write of https://github.com/crisap94/MHZ19 to use Hardware serial port with callbacks.

Functionality is split into two main classes:

:cpp:class:`MHZ19::Uart`
   Access the sensor via serial port

:cpp:class:`MHZ19::PwmReader`
   Asynchronously decodes PWM signal from sensor


ESP8266 Connections
-------------------

Communication requires use of UART0, as UART1 is output-only.

The samples in this library reconfigure the default ``Serial`` class to use UART1
for debug output.

UART0 is reconfigured to use the alternate pins by calling :cpp:func:`HardwareSerial::swap`.
Connect as follows:

====     ===========    =======     ===================
GPIO     Alternate      NodeMCU     Notes
====     ===========    =======     ===================
1        TXD0           TX          } Debug serial output
2        TX1            D4          } Serial1 output
13       RXD2           D7          Serial input from MHZ19
15       TXD2           D8          Serial output to MHZ19
14                      D5          PWM input from MHZ19
====     ===========    =======     ===================

Debug output can be restored by connecting a jumper between GPIO #2 and #1.

Note: This approach is also used for I2S, see :library:`ToneGenerator`.


References
----------

- `MH-Z19 datasheet <http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf>`__
- `MH-Z19B datasheet <https://www.winsen-sensor.com/d/files/MH-Z19B.pdf>`__
- `MH-Z19C datasheet <https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19c-pins-type-co2-manual-ver1_0.pdf>`__


API Documentation
-----------------

.. doxygennamespace:: MHZ19
   :members:
