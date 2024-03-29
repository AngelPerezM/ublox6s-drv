# ublox6s-drv
_Simple driver for uBLOX-LEA-6S GPS's using termios_

This repo contains a C++ driver for GPS's that implements the uBLOX-LEA-6S protocol.
The driver uses the `termios` POSIX library to read and control the GPS through the serial port.

The high-level API implements an asynchronous thread that reads the NMEA data,
and provides the decoded message to the user via a callback.
The minimalistic `minemea` parser, implemented in C, is used to decode NMEA messages.

The GPS supports the configuration of dynamic platform models like pedestrian, airborne < 1G, and automotive, among others.
This software was successfully used in the HERCCULES balloning project during the BEXUS-32 campaign,
reaching about 30 Km of altitude (only possible if configured as airborne 1G).

# Project's structure

- [*`include`*](include): Contains the public header of the driver.
- [*`src`*](src): Contains the implementation of the driver and additional private classes.
