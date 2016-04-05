# MagneticEncoder
## Overview
The magnetic breakout is a breakout PCB for the AMS AS5311 magnetic encoder IC, for testing and development purposes. An external microcontroller is required to track the encoder's position.

The magnetic module combines the features of the breakout board along with an on-board AVR tasked with reading the encoder data and tracking the encoder's position. It exposes the tracked position over an I2C interface.

AMS have now discontinued the AS5311 IC I have been using, so I am switching to the NSE5310 - which I believe AMS intend as a replacement for the AS5311, as it has very similar capabilities. The NSE5310 module is still untested however.

## License
This work is licensed under a [Creative Commons Attribution-NonCommercial 4.0 International License](http://creativecommons.org/licenses/by-nc/4.0/).
