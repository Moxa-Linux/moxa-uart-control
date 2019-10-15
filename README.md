# moxa-uart-control

`moxa-uart-control` is a C library for getting and setting UART ports mode.

## Build

This project use autotools as buildsystem. You can build this project by the following commands:

* If the build target architecture is x86_64

	```
	# ./autogen.sh --host=x86_64-linux-gnu --includedir=/usr/include/moxa --libdir=/usr/lib/x86_64-linux-gnu --sbindir=/sbin
	# make
	# make install
	```
* If the build target architecture is armhf

	```
	# ./autogen.sh --host=arm-linux-gnueabihf --includedir=/usr/include/moxa --libdir=/usr/lib/arm-linux-gnueabihf --sbindir=/sbin
	# make
	# make install
	```

The autogen script will execute ./configure and pass all the command-line
arguments to it.

## Usage of mx-uart-ctl

```
Usage:
	mx-uart-ctl -p <port_number> [-m <uart_mode>]

OPTIONS:
	-p <port_number>
		Set target port.
	-m <uart_mode>
		Set target port to uart_mode
		0 --> set to RS232 mode
		1 --> set to RS485-2W mode
		2 --> set to RS422/RS485-4W mode

Example:
	Get mode from port 0
	# mx-uart-ctl -p 0

	Set port 1 to mode RS232
	# mx-uart-ctl -p 1 -m 0
```

## Documentation

[Config Example](/Config_Example.md)

[API Reference](/API_References.md)
