## Config Example

### Path 
```
/etc/moxa-configs/moxa-uart-control.json
```

### Description

* `CONFIG_VERSION`: The version of config file
* `METHOD`: The method to manipulate UART, including GPIO, IOCTL, and GPIO_IOCTL
* `NUM_OF_UART_PORTS`: The number of UART ports on this device
* `UART_PORTS`: The corresponding TTY name for each UART port
* `GPIO_PINS_PER_UART_PORT`: The number of GPIO pins used by one UART port
* `GPIO_NUMS_OF_UART_PORTS`: The GPIO pin numbers used by each UART port

### Example1: UC-8580

```
{
	"CONFIG_VERSION": "1.1.0",

	"METHOD": "IOCTL",

	"NUM_OF_UART_PORTS": 2,
	"UART_PORTS": [
		"/dev/ttyM0",
		"/dev/ttyM1"
	]
}
```

### Example2: UC-5111-LX

```
{
	"CONFIG_VERSION":"1.1.1",

	"METHOD": "GPIO",

	"NUM_OF_UART_PORTS": 4,
	"UART_PORTS": [
		"/dev/ttyM0",
		"/dev/ttyM1",
		"/dev/ttyM2",
		"/dev/ttyM3"
	],

	"GPIO_PINS_PER_UART_PORT": 4,
	"GPIO_NUMS_OF_UART_PORTS": [
		[496, 497, 498, 499],
		[500, 501, 502, 503],
		[504, 505, 506, 507],
		[508, 509, 510, 511]
	],
	"NUM_OF_UART_MODE": 3,
	"UART_MODES": [
		[1, 1, 0, 0],
		[0, 0, 0, 1],
		[0, 0, 1, 0]
	]
}
```
