/*
 * This file is to define Moxa CPU embedded device private ioctrl command.
 * It is include UART operating mode (RS232, RS422, RS485-2wire, RS485-4wires).
 * UART device node is /dev/ttyM0 - /dev/ttyM3.
 * Copyright Moxa Systems L.T.D.
 * History:
 * Date		Author			Comment
 * 02-20-2006	Victor Yu.		Create it.
 * 09-08-2017	Elvis Yao.		Modify for load config ver.
 */

#ifndef	_MOXA_UART_CTL_H
#define	_MOXA_UART_CTL_H

#define	ARRAY_SIZE(x)	(sizeof(x)/sizeof(x[0]))

/* operating mode value define */
#define	RS232_MODE		0
#define	RS485_2W_MODE		1
#define	RS422_RS485_4W_MODE	2

#define	GET_MODE		0
#define	SET_MODE		1

#define	MAX_BUFF_LEN		256
#define MAX_GPIO_LIST		16
#define MAX_GPIO_PIN		4
#define	MAX_DEV_NODE		32
#define	MAX_GPIO_NODE		64
#define MAX_GPIO_MODE		3
#define	MAX_H_L_LEN		1
#define	GPIO_PATH		"/sys/class/gpio/gpio"
#define	GPIO_VALUE		"/value"
#define	GPIO_DIRECTION		"/direction"
#define	CONFIG_NAME		"/etc/moxa-configs/moxa-uart-control.json"
#define	GPIO_DIRECTION_OUT	"out"

#ifdef	__cplusplus
extern	"C" {
#endif

void show_uart_mode_message(int interface);
void check_device_node(char *device_node);
void check_uart_mode(int uart_mode);
void get_serial_ioctl(char *device, int *mode);
void set_serial_ioctl(char *device, int mode);
int get_serial_gpio(int gpio_num);
int set_serial_gpio(int gpio_num, int low_high);

typedef struct config_struct {
	int ctl_mode;
	int max_uart_port;
	int max_uart_pin;
	int max_uart_mode;
	int set_uart_mode;
	int device_node_len;
	char method[MAX_BUFF_LEN];
	char device_node[MAX_BUFF_LEN];
	char *device_node_list[MAX_DEV_NODE];
	int gpio_list[MAX_GPIO_LIST][MAX_GPIO_PIN];
	int gpio_mode[MAX_GPIO_MODE][MAX_GPIO_PIN];
} CONFIG_STRUCT;

#ifdef __cplusplus
}
#endif

#endif	/* _MOXA_UART_CTL_H */
