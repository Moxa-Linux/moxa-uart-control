/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Name:
 *	MOXA UART Library
 *
 * Description:
 *	Library for getting or setting UART mode.
 *
 * Authors:
 *	2005	Victor Yu	<victor.yu@moxa.com>
 *	2008	CF Lin
 *	2017	Elvis CW Yao	<ElvisCW.Yao@moxa.com>
 *	2018	Ken CJ Chou	<KenCJ.Chou@moxa.com>
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/termios.h>
#include <termio.h>
#include <linux/serial.h>
#include <json-c/json.h>
#include <moxa/mx_gpio.h>
#include <mx_uart.h>

#define CONF_FILE "/etc/moxa-configs/moxa-uart-control.json"
#define CONF_VER_SUPPORTED "1.1.*"

#define CMSPAR 010000000000
#define MAX_FILEPATH_LEN 256	/* reserved length for file path */
#define MAX_BUFFER_LEN 32	/* reserved length for buffer */

struct uart_port_struct {
	int fd;
	int is_opened;
};

struct baudtab_struct {
	int baudrate;
	int index;
};

static int lib_initialized;
static struct json_object *config;
static struct uart_port_struct *uart_ports;
static int def_uart_modes_values[3][4] = {
	{1, 1, 0, 0}, /* RS232 */
	{0, 0, 0, 1}, /* RS485_2W */
	{0, 0, 1, 0}  /* RS422_RS485_4W */
};
/*
 * do not change the order and cancel any one
 */
static struct baudtab_struct baudrate_table[] = {
	{ 0, B0 },
	{ 50, B50 },
	{ 75, B75 },
	{ 110, B110 },
	{ 134, B134 },
	{ 150, B150 },
	{ 200, B200 },
	{ 300, B300 },
	{ 600, B600 },
	{ 1200, B1200 },
	{ 1800, B1800 },
	{ 2400, B2400 },
	{ 4800, B4800 },
	{ 9600, B9600 },
	{ 19200, B19200 },
	{ 38400, B38400 },
	{ 57600, B57600 },
	{ 115200, B115200 },
	{ 230400, B230400 },
	{ 460800, B460800 },
	{ 500000, B500000 },
	{ 576000, B576000 },
	{ 921600, B921600 },
};
static int baudtab_size = sizeof(baudrate_table) / sizeof(baudrate_table[0]);
extern char mx_errmsg[256];

/*
 * json-c utilities
 */

static inline int obj_get_obj(struct json_object *obj, char *key, struct json_object **val)
{
	if (!json_object_object_get_ex(obj, key, val)) {
		sprintf(mx_errmsg, "json-c: can\'t get key: \"%s\"", key);
		return -1;
	}
	return 0;
}

static int obj_get_int(struct json_object *obj, char *key, int *val)
{
	struct json_object *tmp;

	if (obj_get_obj(obj, key, &tmp) < 0)
		return -1;

	*val = json_object_get_int(tmp);
	return 0;
}

static int obj_get_str(struct json_object *obj, char *key, const char **val)
{
	struct json_object *tmp;

	if (obj_get_obj(obj, key, &tmp) < 0)
		return -1;

	*val = json_object_get_string(tmp);
	return 0;
}

static int obj_get_arr(struct json_object *obj, char *key, struct array_list **val)
{
	struct json_object *tmp;

	if (obj_get_obj(obj, key, &tmp) < 0)
		return -1;

	*val = json_object_get_array(tmp);
	return 0;
}

static int arr_get_obj(struct array_list *arr, int idx, struct json_object **val)
{
	if (arr == NULL || idx >= arr->length) {
		sprintf(mx_errmsg, "json-c: can\'t get index: %d", idx);
		return -1;
	}

	*val = array_list_get_idx(arr, idx);
	return 0;
}

static int arr_get_int(struct array_list *arr, int idx, int *val)
{
	struct json_object *tmp;

	if (arr_get_obj(arr, idx, &tmp) < 0)
		return -1;

	*val = json_object_get_int(tmp);
	return 0;
}

static int arr_get_str(struct array_list *arr, int idx, const char **val)
{
	struct json_object *tmp;

	if (arr_get_obj(arr, idx, &tmp) < 0)
		return -1;

	*val = json_object_get_string(tmp);
	return 0;
}

static int arr_get_arr(struct array_list *arr, int idx, struct array_list **val)
{
	struct json_object *tmp;

	if (arr_get_obj(arr, idx, &tmp) < 0)
		return -1;

	*val = json_object_get_array(tmp);
	return 0;
}

/*
 * static functions
 */

static int check_config_version_supported(const char *conf_ver)
{
	int cv[2], sv[2];

	if (sscanf(conf_ver, "%d.%d.%*s", &cv[0], &cv[1]) < 0) {
		sprintf(mx_errmsg, "sscanf: %s: %s", conf_ver, strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (sscanf(CONF_VER_SUPPORTED, "%d.%d.%*s", &sv[0], &sv[1]) < 0) {
		sprintf(mx_errmsg, "sscanf: %s: %s", CONF_VER_SUPPORTED, strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (cv[0] != sv[0] || cv[1] != sv[1]) {
		sprintf(mx_errmsg, "Config version not supported, need to be %s", CONF_VER_SUPPORTED);
		return -4; /* E_UNSUPCONFVER */
	}
	return 0;
}

static int get_uart_port_ttyname(int port, const char **ttyname)
{
	struct array_list *uart_port_ttynames;

	if (obj_get_arr(config, "UART_PORTS", &uart_port_ttynames) < 0)
		return -5; /* E_CONFERR */

	if (arr_get_str(uart_port_ttynames, port, ttyname) < 0)
		return -5; /* E_CONFERR */

	return 0;
}

static int get_uart_mode_ioctl(int port, int *mode)
{
	struct serial_struct serial;
	const char *ttyname;
	int ret, fd;

	ret = get_uart_port_ttyname(port, &ttyname);
	if (ret < 0)
		return ret;

	fd = open(ttyname, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		sprintf(mx_errmsg, "open %s: %s", ttyname, strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	serial.reserved_char[0] = 0;
	if (ioctl(fd, TIOCGSERIAL, &serial) < 0) {
		sprintf(mx_errmsg, "ioctl: TIOCGSERIAL: %s", strerror(errno));
		close(fd);
		return -1; /* E_SYSFUNCERR */
	}
	close(fd);

	*mode = serial.port;
	return 0;
}

static int set_uart_mode_ioctl(int port, int mode)
{
	struct serial_struct serial;
	const char *ttyname;
	int ret, fd;

	ret = get_uart_port_ttyname(port, &ttyname);
	if (ret < 0)
		return ret;

	fd = open(ttyname, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		sprintf(mx_errmsg, "open %s: %s", ttyname, strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (ioctl(fd, TIOCGSERIAL, &serial) < 0) {
		sprintf(mx_errmsg, "ioctl: TIOCGSERIAL: %s", strerror(errno));
		close(fd);
		return -1; /* E_SYSFUNCERR */
	}

	serial.port = mode;
	if (ioctl(fd, TIOCSSERIAL, &serial) < 0) {
		sprintf(mx_errmsg, "ioctl: TIOCSSERIAL: %s", strerror(errno));
		close(fd);
		return -1; /* E_SYSFUNCERR */
	}
	close(fd);

	return 0;
}

static int init_gpio_pin(int gpio_num)
{
	int ret;

	if (mx_gpio_is_exported(gpio_num))
		return 0;

	ret = mx_gpio_export(gpio_num);
	if (ret < 0)
		return ret;

	ret = mx_gpio_set_direction(gpio_num, GPIO_DIRECTION_OUT);
	if (ret < 0)
		return ret;

	return 0;
}

static int init_gpio(int port)
{
	struct array_list *gpio_nums_of_uart_ports, *gpio_nums;
	int ret, gpio_num, num_of_gpio_pins, i;

	if (obj_get_arr(config, "GPIO_NUMS_OF_UART_PORTS", &gpio_nums_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (arr_get_arr(gpio_nums_of_uart_ports, port, &gpio_nums) < 0)
		return -5; /* E_CONFERR */

	if (obj_get_int(config, "GPIO_PINS_PER_UART_PORT", &num_of_gpio_pins) < 0)
		return -5; /* E_CONFERR */

	for (i = 0; i < num_of_gpio_pins; i++) {
		if (arr_get_int(gpio_nums, i, &gpio_num) < 0)
			return -5; /* E_CONFERR */

		ret = init_gpio_pin(gpio_num);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int get_uart_modes(int mode, int **uart_modes_values)
{
	struct array_list *uart_modes, *uart_mode_pins;
	int num_of_gpio_pins, i;
	int *tmp_uart_modes_values;

	if (obj_get_arr(config, "UART_MODES", &uart_modes) < 0) {
		*uart_modes_values = (int *) def_uart_modes_values[mode];
		return 0;
	}

	if (obj_get_int(config, "GPIO_PINS_PER_UART_PORT", &num_of_gpio_pins) < 0)
		return -5; /* E_CONFERR */

	tmp_uart_modes_values = (int *) malloc(num_of_gpio_pins * sizeof(int));
	if (tmp_uart_modes_values == NULL) {
		sprintf(mx_errmsg, "malloc: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (arr_get_arr(uart_modes, mode, &uart_mode_pins) < 0)
		return -5; /* E_CONFERR */

	for (i = 0; i < num_of_gpio_pins; i++) {
		if (arr_get_int(uart_mode_pins, i, &tmp_uart_modes_values[i]) < 0)
			return -5; /* E_CONFERR */
	}

	*uart_modes_values = tmp_uart_modes_values;
	return 0;
}

static int arrncmp(int num, int *a, int *b)
{
	int i, match = 0;

	for (i = 0; i < num; i++) {
		if (a[i] != b[i])
			match = 1;
	}
	return match;
}

static int set_uart_mode_gpio(int port, int mode)
{
	struct array_list *gpio_nums_of_uart_ports, *gpio_nums;
	int ret, num_of_gpio_pins, gpio_num, i;
	int *uart_modes_values;

	ret = init_gpio(port);
	if (ret < 0)
		return ret;

	if (obj_get_arr(config, "GPIO_NUMS_OF_UART_PORTS", &gpio_nums_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (arr_get_arr(gpio_nums_of_uart_ports, port, &gpio_nums) < 0)
		return -5; /* E_CONFERR */

	if (obj_get_int(config, "GPIO_PINS_PER_UART_PORT", &num_of_gpio_pins) < 0)
		return -5; /* E_CONFERR */

	ret = get_uart_modes(mode, &uart_modes_values);
	if (ret < 0)
		return ret;

	for (i = 0; i < num_of_gpio_pins; i++) {
		if (arr_get_int(gpio_nums, i, &gpio_num) < 0)
			return -5; /* E_CONFERR */

		ret = mx_gpio_set_value(gpio_num, uart_modes_values[i]);
		if (ret < 0)
			return ret;
	}
	if (uart_modes_values != def_uart_modes_values[mode])
		free(uart_modes_values);
	return 0;
}

static int get_uart_mode_gpio(int port, int *mode)
{
	struct array_list *gpio_nums_of_uart_ports, *gpio_nums;
	int ret, num_of_gpio_pins, gpio_num, gpio_value, i;
	int *gpio_values;
	int *uart_modes_values;

	ret = init_gpio(port);
	if (ret < 0)
		return ret;

	if (obj_get_arr(config, "GPIO_NUMS_OF_UART_PORTS", &gpio_nums_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (arr_get_arr(gpio_nums_of_uart_ports, port, &gpio_nums) < 0)
		return -5; /* E_CONFERR */

	if (obj_get_int(config, "GPIO_PINS_PER_UART_PORT", &num_of_gpio_pins) < 0)
		return -5; /* E_CONFERR */

	gpio_values = (int *) malloc(num_of_gpio_pins * sizeof(int));
	if (gpio_values == NULL) {
		sprintf(mx_errmsg, "malloc: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	for (i = 0; i < num_of_gpio_pins; i++) {
		if (arr_get_int(gpio_nums, i, &gpio_num) < 0)
			return -5; /* E_CONFERR */

		ret = mx_gpio_get_value(gpio_num, &gpio_value);
		if (ret < 0)
			return ret;

		gpio_values[i] = gpio_value;
	}

	/* compare to uart mode */
	for (i = 0; i < 3; i++) {
		ret = get_uart_modes(i, &uart_modes_values);
		if (ret < 0)
			return ret;

		if (!arrncmp(num_of_gpio_pins, gpio_values, uart_modes_values)) {
			*mode = i;
			return 0;
		}

		if (uart_modes_values != def_uart_modes_values[i])
			free(uart_modes_values);
	}

	sprintf(mx_errmsg, "Unknown UART mode");
	return -52; /* E_UART_UNKMODE */
}

static int get_uart_mode_gpio_ioctl(int port, int *mode)
{
	int ret, mode_from_gpio, mode_from_ioctl;

	ret = get_uart_mode_gpio(port, &mode_from_gpio);
	if (ret < 0)
		return ret;
	ret = get_uart_mode_ioctl(port, &mode_from_ioctl);
	if (ret < 0)
		return ret;

	if (mode_from_gpio != mode_from_ioctl) {
		sprintf(mx_errmsg, "GPIO and IOCTL are incompatible");
		return -51; /* E_UART_GPIOIOCTLINCOMP */
	}

	*mode = mode_from_gpio;
	return 0;
}

static int set_uart_mode_gpio_ioctl(int port, int mode)
{
	int ret;

	ret = set_uart_mode_gpio(port, mode);
	if (ret < 0)
		return ret;
	ret = set_uart_mode_ioctl(port, mode);
	if (ret < 0)
		return ret;

	return get_uart_mode_gpio_ioctl(port, &mode);
}

/*
 * APIs
 */

int mx_uart_init(void)
{
	int ret, num_of_uart_ports, i;
	const char *conf_ver;


	if (lib_initialized)
		return 0;

	config = json_object_from_file(CONF_FILE);
	if (config == NULL)
		return -5; /* E_CONFERR */

	if (obj_get_str(config, "CONFIG_VERSION", &conf_ver) < 0)
		return -5; /* E_CONFERR */

	ret = check_config_version_supported(conf_ver);
	if (ret < 0)
		return ret;

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	uart_ports = (struct uart_port_struct *)
		malloc(num_of_uart_ports * sizeof(struct uart_port_struct));
	if (uart_ports == NULL) {
		sprintf(mx_errmsg, "malloc: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	for (i = 0; i < num_of_uart_ports; i++)
		uart_ports[i].is_opened = 0;

	lib_initialized = 1;
	return 0;
}

int mx_uart_set_mode(int port, int mode)
{
	int num_of_uart_ports;
	const char *method;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (mode != UART_MODE_RS232 && mode != UART_MODE_RS485_2W
		&& mode != UART_MODE_RS422_RS485_4W) {
		sprintf(mx_errmsg, "Try to set unknown mode: %d", mode);
		return -2; /* E_INVAL */
	}

	if (obj_get_str(config, "METHOD", &method) < 0)
		return -5; /* E_CONFERR */

	if (strcmp(method, "IOCTL") == 0)
		return set_uart_mode_ioctl(port, mode);
	else if (strcmp(method, "GPIO") == 0)
		return set_uart_mode_gpio(port, mode);
	else if (strcmp(method, "GPIO_IOCTL") == 0)
		return set_uart_mode_gpio_ioctl(port, mode);

	return -5; /* E_CONFERR */
}

int mx_uart_get_mode(int port, int *mode)
{
	int num_of_uart_ports;
	const char *method;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (obj_get_str(config, "METHOD", &method) < 0)
		return -5; /* E_CONFERR */

	if (strcmp(method, "IOCTL") == 0)
		return get_uart_mode_ioctl(port, mode);
	else if (strcmp(method, "GPIO") == 0)
		return get_uart_mode_gpio(port, mode);
	else if (strcmp(method, "GPIO_IOCTL") == 0)
		return get_uart_mode_gpio_ioctl(port, mode);

	return -5; /* E_CONFERR */
}

int mx_uart_open(int port)
{
	struct termios tmio;
	const char *ttyname;
	int ret, num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (uart_ports[port].is_opened == 1)
		return 0;

	ret = get_uart_port_ttyname(port, &ttyname);
	if (ret < 0)
		return ret;

	uart_ports[port].fd = open(ttyname, O_RDWR|O_NOCTTY);
	if (uart_ports[port].fd < 0) {
		sprintf(mx_errmsg, "open %s: %s", ttyname, strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	flock(uart_ports[port].fd, LOCK_EX);

	tcflush(uart_ports[port].fd, TCIOFLUSH);
	tcgetattr(uart_ports[port].fd, &tmio);
	tmio.c_lflag = 0;
	tmio.c_oflag = 0;
	tmio.c_iflag = 0;
	tmio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
	tmio.c_cc[VTIME] = 0;
	tmio.c_cc[VMIN] = 0;
	tcsetattr(uart_ports[port].fd, TCSANOW, &tmio);

	uart_ports[port].is_opened = 1;
	return 0;
}

int mx_uart_close(int port)
{
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (uart_ports[port].is_opened == 0)
		return 0;

	close(uart_ports[port].fd);
	uart_ports[port].is_opened = 0;
	return 0;
}

int mx_uart_read(int port, char *data, size_t count)
{
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (uart_ports[port].is_opened == 0) {
		sprintf(mx_errmsg, "UART port %d is not opened", port);
		return -50; /* E_UART_NOTOPEN */
	}

	return read(uart_ports[port].fd, data, count);
}

int mx_uart_write(int port, char *data, size_t count)
{
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (uart_ports[port].is_opened == 0) {
		sprintf(mx_errmsg, "UART port %d is not opened", port);
		return -50; /* E_UART_NOTOPEN */
	}

	return write(uart_ports[port].fd, data, count);
}

int mx_uart_set_baudrate(int port, int baudrate)
{
	struct termios termios;
	int i, num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	/* check and setup configuration */
	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	/* check if the speed is one of the well-known baud rates */
	for (i = 0; i < baudtab_size; i++) {
		if (baudrate_table[i].baudrate == baudrate) {
			termios.c_cflag &= ~(CBAUD | CBAUDEX);
			termios.c_cflag |= baudrate_table[i].index;
			if (tcsetattr(uart_ports[port].fd, TCSANOW, &termios)) {
				sprintf(mx_errmsg, "tcsetattr: %s", strerror(errno));
				return -1; /* E_SYSFUNCERR */
			}
			return 0;
		}
	}

	/* a rate not one of them defined in the table */
	termios.c_cflag |= B4000000;
	if (ioctl(uart_ports[port].fd, UC_SET_SPECIAL_BAUD_RATE, &baudrate)) {
		sprintf(mx_errmsg, "ioctl: UC_SET_SPECIAL_BAUD_RATE: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	return 0;
}

int mx_uart_get_baudrate(int port, int *baudrate)
{
	struct termios termios;
	int i, index, num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	/* check and setup configuration */
	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	/* match one in the table */
	index = termios.c_cflag & CBAUD;
	for (i = 0; i < baudtab_size; i++) {
		if (baudrate_table[i].index == index) {
			*baudrate = baudrate_table[i].baudrate;
			return 0;
		}
	}

	if (ioctl(uart_ports[port].fd, UC_GET_SPECIAL_BAUD_RATE, baudrate) != 0) {
		sprintf(mx_errmsg, "ioctl: UC_GET_SPECIAL_BAUD_RATE: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	return 0;
}

int mx_uart_set_databits(int port, int bits)
{
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	termios.c_cflag &= ~CSIZE;
	if (bits == 5)
		termios.c_cflag |= CS5;
	else if (bits == 6)
		termios.c_cflag |= CS6;
	else if (bits == 7)
		termios.c_cflag |= CS7;
	else
		termios.c_cflag |= CS8;

	if (tcsetattr(uart_ports[port].fd, TCSANOW, &termios)) {
		sprintf(mx_errmsg, "tcsetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	return 0;
}

int mx_uart_get_databits(int port, int *bits)
{
	unsigned int flag;
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	flag = termios.c_cflag & CSIZE;

	if (flag == CS8)
		*bits = 8;
	else if (flag == CS7)
		*bits = 7;
	else if (flag == CS6)
		*bits = 6;
	else if (flag == CS5)
		*bits = 5;

	return 0;
}

int mx_uart_set_stopbits(int port, int bits)
{
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (bits == 2)
		termios.c_cflag |= CSTOPB;
	else
		termios.c_cflag &= ~CSTOPB;

	if (tcsetattr(uart_ports[port].fd, TCSANOW, &termios)) {
		sprintf(mx_errmsg, "tcsetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	return 0;
}

int mx_uart_get_stopbits(int port, int *bits)
{
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	/* check and setup configuration */
	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (termios.c_cflag & CSTOPB)
		*bits = 2;
	else
		*bits = 1;

	return 0;
}

int mx_uart_set_parity(int port, int parity)
{
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	/* check and setup configuration */
	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	termios.c_cflag &= ~(PARENB|PARODD|CMSPAR);

	if (parity == MSP_PARITY_ODD)
		termios.c_cflag |= (PARENB|PARODD);
	else if (parity == MSP_PARITY_EVEN)
		termios.c_cflag |= PARENB;
	else if (parity == MSP_PARITY_SPACE)
		termios.c_cflag |= (PARENB|CMSPAR);
	else if (parity == MSP_PARITY_MARK)
		termios.c_cflag |= (PARENB|CMSPAR|PARODD);

	if (tcsetattr(uart_ports[port].fd, TCSANOW, &termios)) {
		sprintf(mx_errmsg, "tcsetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}
	return 0;
}

int mx_uart_get_parity(int port, int *parity)
{
	struct termios termios;
	int num_of_uart_ports;

	if (!lib_initialized) {
		sprintf(mx_errmsg, "Library is not initialized");
		return -3; /* E_LIBNOTINIT */
	}

	if (obj_get_int(config, "NUM_OF_UART_PORTS", &num_of_uart_ports) < 0)
		return -5; /* E_CONFERR */

	if (port < 0 || port >= num_of_uart_ports) {
		sprintf(mx_errmsg, "UART port out of index: %d", port);
		return -2; /* E_INVAL */
	}

	/* check and setup configuration */
	if (tcgetattr(uart_ports[port].fd, &termios)) {
		sprintf(mx_errmsg, "tcgetattr: %s", strerror(errno));
		return -1; /* E_SYSFUNCERR */
	}

	if (termios.c_cflag & PARENB) {
		if (termios.c_cflag & CMSPAR)
			*parity = (termios.c_cflag & PARODD) ? MSP_PARITY_MARK : MSP_PARITY_SPACE;
		else
			*parity = (termios.c_cflag & PARODD) ? MSP_PARITY_ODD : MSP_PARITY_EVEN;
	} else {
		*parity = MSP_PARITY_NONE;
	}
	return 0;
}
