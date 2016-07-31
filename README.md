# MagneticEncoder
## Overview
The magnetic breakout is a breakout PCB for the AMS AS5311 magnetic encoder IC, for testing and development purposes. An external microcontroller is required to track the encoder's position.

The magnetic module combines the features of the breakout board along with an on-board AVR tasked with reading the encoder data and tracking the encoder's position. It exposes the tracked position over an I2C interface.

AMS have now discontinued the AS5311 IC I have been using, so I am switching to the NSE5310 - which I believe AMS intend as a replacement for the AS5311, as it has very similar capabilities. The NSE5310 module is still untested however.

## License
In the spirit of the RepRap project, the magnetic encoder module is fully open source - files to manufacture or modify a board of your own are included in this repository. This project is licensed under a GPLv3 license, and anyone is free to modify it, manufacture it, sell it - as long as you share your modifications under the same license, we're good!
