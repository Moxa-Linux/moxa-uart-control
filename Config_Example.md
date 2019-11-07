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

### Example3: MC-1200

```
{
    "CONFIG_VERSION": "1.1.2",
    "METHOD": "FILEPATH",
    "NUM_OF_UART_PORTS": 2,
    "UART_PORTS": [
            "/dev/ttyM0",
            "/dev/ttyM1"
    ],
    "UART_PORTS_GROUP": [
            "1",
            "1"
    ],
    "FILEPATH_PER_UART_PORT": 4,
    "FILEPATH_OF_UART_PORTS": [
        [
            "/sys/class/gpio/gpio449/value",
            "/sys/class/gpio/gpio450/value",
            "/sys/class/gpio/gpio451/value",
            "/sys/class/misc/it87_serial/serial1/serial1_rs485"
        ],
        [
            "/sys/class/gpio/gpio452/value",
            "/sys/class/gpio/gpio453/value",
            "/sys/class/gpio/gpio454/value",
            "/sys/class/misc/it87_serial/serial2/serial2_rs485"
        ]
    ],
    "NUM_OF_UART_MODE": 3,
    "UART_MODES_GROUP1": [
            [0, 0, 1, 0],
            [1, 0, 0, 1],
            [0, 1, 0, 1]
    ]
}
```

### Example4: DA-820C
- On board it8786: /dev/ttyM0 ~ /dev/ttyM1
- Expansion UART board: /dev/ttyM2 ~ /dev/ttyM9

```
{
    "CONFIG_VERSION": "1.1.2",
    "METHOD": "FILEPATH",
    "NUM_OF_UART_PORTS": 10,
    "UART_PORTS": [
            "/dev/ttyM0",
            "/dev/ttyM1",
            "/dev/ttyM2",
            "/dev/ttyM3",
            "/dev/ttyM4",
            "/dev/ttyM5",
            "/dev/ttyM6",
            "/dev/ttyM7",
            "/dev/ttyM8",
            "/dev/ttyM9"
    ],
    "UART_PORTS_GROUP": [
            "1",
            "1",
            "2",
            "2",
            "2",
            "2",
            "2",
            "2",
            "2",
            "2"
    ],
    "FILEPATH_PER_UART_PORT": 4,
    "FILEPATH_OF_UART_PORTS": [
        [
            "/sys/class/gpio/gpio451/value",
            "/sys/class/gpio/gpio449/value",
            "/sys/class/gpio/gpio450/value",
            "/sys/class/misc/it87_serial/serial1/serial1_rs485"
        ],
        [
            "/sys/class/gpio/gpio454/value",
            "/sys/class/gpio/gpio452/value",
            "/sys/class/gpio/gpio453/value",
            "/sys/class/misc/it87_serial/serial2/serial2_rs485"
        ],
        [
            "/sys/class/gpio/gpio432/value",
            "/sys/class/gpio/gpio433/value",
            "/sys/class/gpio/gpio434/value",
            "/sys/class/gpio/gpio435/value"
        ],
        [
            "/sys/class/gpio/gpio436/value",
            "/sys/class/gpio/gpio437/value",
            "/sys/class/gpio/gpio438/value",
            "/sys/class/gpio/gpio439/value"
        ],
        [
            "/sys/class/gpio/gpio440/value",
            "/sys/class/gpio/gpio441/value",
            "/sys/class/gpio/gpio442/value",
            "/sys/class/gpio/gpio443/value"
        ],
        [
            "/sys/class/gpio/gpio444/value",
            "/sys/class/gpio/gpio445/value",
            "/sys/class/gpio/gpio446/value",
            "/sys/class/gpio/gpio447/value"
        ],
        [
            "/sys/class/gpio/gpio416/value",
            "/sys/class/gpio/gpio417/value",
            "/sys/class/gpio/gpio418/value",
            "/sys/class/gpio/gpio419/value"
        ],
        [
            "/sys/class/gpio/gpio420/value",
            "/sys/class/gpio/gpio421/value",
            "/sys/class/gpio/gpio422/value",
            "/sys/class/gpio/gpio423/value"
        ],
        [
            "/sys/class/gpio/gpio424/value",
            "/sys/class/gpio/gpio425/value",
            "/sys/class/gpio/gpio426/value",
            "/sys/class/gpio/gpio427/value"
        ],
        [
            "/sys/class/gpio/gpio428/value",
            "/sys/class/gpio/gpio429/value",
            "/sys/class/gpio/gpio430/value",
            "/sys/class/gpio/gpio431/value"
        ]
    ],
    "NUM_OF_UART_MODE": 3,
    "UART_MODES_GROUP1": [
            [1, 0, 0, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 1]
    ],
    "UART_MODES_GROUP2": [
            [1, 1, 0, 0],
            [0, 0, 0, 1],
            [0, 0, 1, 0]
    ]
}
```

### Example5: UC-8410A

```
{
	"CONFIG_VERSION":"1.1.3",
	
	"METHOD": "IOCTL_MU860",
	
	"NUM_OF_UART_PORTS": 8,
	"UART_PORTS": [
		"/dev/ttyM0",
		"/dev/ttyM1",
		"/dev/ttyM2",
		"/dev/ttyM3",
		"/dev/ttyM4",
		"/dev/ttyM5",
		"/dev/ttyM6",
		"/dev/ttyM7"
	]
}
```
