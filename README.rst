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

The samples in this library reconfigure the default ``Serial`` class to use UART1 for debug output.

UART0 is reconfigured to use the alternate pins by calling :cpp:func:`HardwareSerial::swap`.
Connect as follows:

============  =============   ===========================
From          To              Notes
============  =============   ===========================
MHZ19 PWM     GPIO14 (D5)
MHZ19 TX      GPIO13 (D7)     UART0 RX
MHZ19 RX      GPIO15 (D8)     UART0 TX
GPIO2 (D4)    GPIO1 (TX)      Route UART1 output to USB
============  =============   ===========================

Note: A similar approach is used for I2S, see :library:`ToneGenerator`.


References
----------

- `MH-Z19 datasheet <http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z19%20CO2%20Ver1.0.pdf>`__
- `MH-Z19B datasheet <https://www.winsen-sensor.com/d/files/MH-Z19B.pdf>`__
- `MH-Z19C datasheet <https://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19c-pins-type-co2-manual-ver1_0.pdf>`__


API Documentation
-----------------

.. doxygennamespace:: MHZ19
   :members:
