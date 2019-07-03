# API References

---
### int mx_uart_init(void)

Initialize Moxa UART control library.

#### Return value
* 0 on success.
* negative numbers on error.

---
### int mx_uart_set_mode(int port, int mode)

Set state for target Digital Output port.

#### Parameters
* port: target DOUT port number
* mode: UART_MODE_RS232 or UART_MODE_RS485_2W or UART_MODE_RS422_RS485_4W

#### Return value
* 0 on success.
* negative numbers on error.

---
### int mx_uart_get_mode(int port, int *mode)

Get state from target Digital Output port.

#### Parameters
* port: target DOUT port number
* mode: where the output value will be set.

#### Return value
* 0 on success.
* negative numbers on error.

---