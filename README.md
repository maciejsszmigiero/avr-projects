# avr-projects repository

This repository will contain various Atmel AVR microcontroller projects which share common library code.

Currently, there is only one project in this repository: a Smart UPS addon (in smartupsaddon directory).
Please refer to its directory for details.

## Common library code
The library code was written for ATmega 1284(P) µC, but patches to add support for other AVR µC models are welcome.

### Features
* A timekeeping subsystem that allows scheduling events at a particular future time, measuring elapsed time and comparing time intervals.

* An I²C subsystem (with internal transaction watchdog) that has been extensively tested under heavy electrical noise to automatically recover any known stuck I²C bus interface condition.

* A serial port subsystem:
  * With per-port circular buffers for transmit and receive data,

  * With ability to switch one of the chip serial ports into a "debug port" mode and redirect the C library "standard error" output there.
    This debug output is then interleaved with what the application normally outputs to such serial port - marked appropriately with '[' and ']' characters so these two streams can be differentiated.

  * Each serial port can have a different baud rate and differently sized circular buffers. The transmit and receive data buffers themselves can have different sizes, too.

* Uses compiler barriers and interrupt disabling "atomic blocks" where necessary to avoid declaring variables shared with interrupt handlers as volatile.
