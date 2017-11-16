/*
 * To set serial port interface.
 * Usage: mx-uart-ctl	-i	device-node [interface-no]
 *	device-node	-	must be /dev/ttyM0 or /dev/ttyM1
 *	Interface-no	-	following
 *	none - to view now setting
 *	0 - RS232
 *	1 - RS485-2WIRES
 *	2 - RS422/RS485-4WIRES
 *
 * History:
 * Date		Author		Comment
 * 03-28-2005	Victor Yu.	Create it.
 * 09-15-2017	Elvis Yao.	Add GPIO method and modify for loading config.
 */

#include	<stdio.h>
#include	<stdlib.h>
#include	<fcntl.h>
#include	<unistd.h>
#include	<string.h>
#include	<sys/ioctl.h>
#include	<linux/serial.h>
#include	<json-c/json.h>
#include	"mx-uart-ctl.h"

static CONFIG_STRUCT config;

void usage(void)
{
	printf("Usage: mx-uart-ctl -f [device-node] -m [interface-mode]\n");
	printf("\tdevice-node   - /dev/ttyM[0..%d]\n", config.max_uart_port - 1);
	printf("\tinterface-mode  - following:\n");
	printf("\t0 - set to RS232 interface\n");
	printf("\t1 - set to RS485-2W interface\n");
	printf("\t2 - set to RS422/RS485-4W interface\n");
	exit(0);
}

void get_serial_ioctl(char *device, int *mode)
{
	struct	serial_struct serinfo;
	int	fd;

	fd = open(device, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		perror(device);
		return;
	}
	serinfo.reserved_char[0] = 0;

	if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0) {
		perror("Cannot get serial info");
		close(fd);
	return;
	}

	/* People understand 2 better than 9 */
	if (serinfo.irq == 9)
		serinfo.irq = 2;
	*mode = serinfo.port;
	close(fd);
}

void set_serial_ioctl(char *device, int mode)
{
	struct	serial_struct old_serinfo, new_serinfo;
	int	fd;

	fd = open(device, O_RDWR|O_NONBLOCK);
	if (fd < 0) {
		perror(device);
		exit(1);
	}
	if (ioctl(fd, TIOCGSERIAL, &old_serinfo) < 0) {
		perror("Cannot get serial info");
		exit(1);
	}

	new_serinfo = old_serinfo;
	new_serinfo.port = mode;
	if (ioctl(fd, TIOCSSERIAL, &new_serinfo) < 0) {
		perror("Cannot set serial info");
		exit(1);
	}
	close(fd);
}

void exec_ioctl_cmd(void)
{
	int interface;
	/* check device node is existed */
	check_device_node(config.device_node);

	/* get serial mode */
	if (config.ctl_mode == GET_MODE) {
		get_serial_ioctl(config.device_node, &interface);
		show_uart_mode_message(interface);
		exit(0);
	} else if (config.ctl_mode == SET_MODE) {
		interface = config.set_uart_mode;
		if (interface != RS232_MODE &&
			interface != RS485_2W_MODE &&
			interface != RS422_RS485_4W_MODE) {
			printf("Error interface mode.\n");
			usage();
		} else {
			set_serial_ioctl(config.device_node, interface);
		}
	}
	show_uart_mode_message(interface);
	exit(0);
}

int diff_array(int *array_a, int *array_b, int arr_len)
{
	int ret = 0, i;

	for (i = 0; i < arr_len; i++) {
		if (array_a[i] != array_b[i])
			ret = -1;
	}
	return ret;
}

void get_gpio_mode(void)
{
	int i, interface, arr_len;
	int port_idx = -1, cmp_ret = -1;
	int pin_arrays[config.max_uart_pin];

	memset(pin_arrays, 0, sizeof(pin_arrays[0])*config.max_uart_pin);

	/* get the device node index */
	for (i = 0; i < config.device_node_len; i++) {
		if (strcmp(config.device_node,
			config.device_node_list[i]) == 0) {
			port_idx = i;
			break;
		}
	}

	if (port_idx < 0) {
		printf("Cannot get serial info from config file.\n");
		usage();
		exit(1);
	}

	/* get gpio device node value */
	for (i = 0; i < config.max_uart_pin; i++)
		pin_arrays[i] = get_serial_gpio(config.gpio_list[port_idx][i]);

	/* compare to uart mode in config file */
	/* arr_len = (int) (sizeof(pin_arrays)/sizeof(pin_arrays[0])); */
	arr_len = ARRAY_SIZE(pin_arrays);
	for (i = 0; i < config.max_uart_mode; i++) {
		cmp_ret = diff_array(pin_arrays, config.gpio_mode[i], arr_len);
		if (cmp_ret == 0) {
			interface = i;
			break;
		}
	}

	/* show current UART setting */
	if (cmp_ret == 0)
		show_uart_mode_message(interface);
	else
		printf("Unknown interface mode.\n");

}

void set_gpio_mode(void)
{
	int i, cur_mode;
	int port_idx = -1;

	cur_mode = config.set_uart_mode;

	/* get the device node index */
	for (i = 0; i < config.device_node_len; i++) {
		if (strcmp(config.device_node,
			config.device_node_list[i]) == 0) {
			port_idx = i;
			break;
		}
	}

	if (port_idx < 0) {
		printf("Cannot get serial info from config file.\n");
		usage();
		exit(1);
	}

	for (i = 0; i < config.max_uart_pin; i++) {
		set_serial_gpio(config.gpio_list[port_idx][i],
			 config.gpio_mode[cur_mode][i]);
	}
}

int get_serial_gpio(int gpio_num)
{
	int ret;
	char gpio_node[MAX_GPIO_NODE];

	memset(gpio_node, 0, sizeof(gpio_node[0])*MAX_GPIO_NODE);

	snprintf(gpio_node, sizeof(gpio_node), "%s%d%s",
		GPIO_PATH, gpio_num, GPIO_VALUE);

	FILE *fp = fopen(gpio_node, "r");

	if (fp) {
		fscanf(fp, "%d", &ret);
	} else {
		printf("Fail to open %s\nProgram exit.\n", gpio_node);
		exit(1);
	}
	fclose(fp);
	return ret;
}

int set_serial_gpio(int gpio_num, int low_high)
{
	int ret;
	char gpio_node[MAX_GPIO_NODE];
	char low_high_str[MAX_H_L_LEN];

	memset(gpio_node, 0, sizeof(gpio_node[0])*MAX_GPIO_NODE);
	memset(low_high_str, 0, sizeof(low_high_str[0])*MAX_H_L_LEN);

	sprintf(low_high_str, "%d", low_high);
	snprintf(gpio_node, sizeof(gpio_node), "%s%d%s",
			GPIO_PATH, gpio_num, GPIO_VALUE);

	FILE *fp = fopen(gpio_node, "w");

	if (fp) {
		ret = fprintf(fp, "%s", low_high_str);
	} else {
		printf("Fail to open %s\nProgram exit.\n", gpio_node);
		exit(1);
	}
	fclose(fp);
	return ret;
}

void gpio_init_action(int gpio_pin)
{
	int fd;
	ssize_t bytes_written;
	char buffer[MAX_GPIO_PIN];
	char gpio_node[MAX_GPIO_NODE];

	memset(buffer, 0, sizeof(buffer[0])*MAX_GPIO_PIN);
	memset(gpio_node, 0, sizeof(gpio_node[0])*MAX_GPIO_NODE);

	snprintf(gpio_node, sizeof(gpio_node), "%s%d%s",
		GPIO_PATH, gpio_pin, GPIO_DIRECTION);

	/* if device node is existed, do nothing */
	fd = open(gpio_node, O_WRONLY);
	if (fd > 0) {
		close(fd);
		return;
	}

	/* if device node is not existed, export it first */
	fd = open("/sys/class/gpio/export", O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open export for writing!\n");
		exit(1);
	}
	bytes_written = snprintf(buffer, MAX_GPIO_PIN, "%d", gpio_pin);
	write(fd, buffer, bytes_written);

	/* and set direction */
	fd = open(gpio_node, O_WRONLY);
	if (fd == -1) {
		fprintf(stderr, "Failed to open gpio direction for writing!\n");
		exit(1);
	}
	write(fd, GPIO_DIRECTION_OUT, sizeof(GPIO_DIRECTION_OUT));
	close(fd);
}

void gpio_init(void)
{
	int i, j;

	for (i = 0; i < config.max_uart_port; i++)
		for (j = 0 ; j < config.max_uart_pin; j++)
			gpio_init_action(config.gpio_list[i][j]);
}

void exec_gpio_cmd(void)
{
	check_device_node(config.device_node);
	if (config.ctl_mode == GET_MODE)
		get_gpio_mode();
	else if (config.ctl_mode == SET_MODE) {
		set_gpio_mode();
		get_gpio_mode();
	} else {
		usage();
		exit(1);
	}
}

void get_jobj_by_filed(struct json_object *input_jobj,
		const char *field, struct json_object *output_jobj)
{
	if (input_jobj == NULL) {
		printf("Input json object isn't exist.\n");
		exit(1);
	}

	if (!json_object_object_get_ex(input_jobj, field, &output_jobj)) {
		printf("Field %s isn't exist.\n", field);
		exit(1);
	}
}

void load_config_file(char *config_filename)
{
	int i, j, arraylen;
	struct json_object *jobj;
	struct json_object *jarray;
	struct json_object *jitem;
	struct json_object *jsubitem;
	char cur_gpio_port[MAX_DEV_NODE];
	char cur_mode[MAX_DEV_NODE];

	memset(cur_gpio_port, 0, sizeof(cur_gpio_port[0])*MAX_DEV_NODE);
	memset(cur_mode, 0, sizeof(cur_mode[0])*MAX_DEV_NODE);

	jobj = json_object_from_file(config_filename);
	if (jobj == NULL) {
		printf("Fail to open config file: %s\nProgram exit.\n",
		config_filename);
		exit(1);
	}

	if (!json_object_object_get_ex(jobj, "METHOD", &jitem)) {
		printf("Get METHOD key error.\n");
		exit(1);
	}
	strcpy(config.method, json_object_get_string(jitem));

	if (!json_object_object_get_ex(jobj, "MAX_GPIO_PORT", &jitem)) {
		printf("Get MAX_GPIO_PORT key error.\n");
		exit(1);
	}
	config.max_uart_port = json_object_get_int(jitem);

	if (strcmp(config.method, "GPIO") == 0) {
		if (!json_object_object_get_ex(jobj, "MAX_GPIO_PIN", &jitem)) {
			printf("Get MAX_GPIO_PIN key error.\n");
			exit(1);
		}
		config.max_uart_pin = json_object_get_int(jitem);

		if (!json_object_object_get_ex(jobj, "MAX_GPIO_MODE", &jitem)) {
			printf("Get MAX_GPIO_MODE key error.\n");
			exit(1);
		}
		config.max_uart_mode = json_object_get_int(jitem);

		if (!json_object_object_get_ex(jobj, "UART_GPIO", &jarray)) {
			printf("Get UART_GPIO key error.\n");
			exit(1);
		}

		/* key and val don't exist outside of this block */
		i = 0;
		config.device_node_len = json_object_object_length(jarray);
		json_object_object_foreach(jarray, key, val) {
			config.device_node_list[i] = key;
			for (j = 0 ; j < json_object_array_length(val); j++) {
				jitem = json_object_array_get_idx(val, j);
				config.gpio_list[i][j] =
					json_object_get_int(jitem);
			}
			i++;
		}

		if (!json_object_object_get_ex(jobj,
			"UART_GPIO_MODE", &jarray)) {
			printf("Get UART_GPIO_MODE key error.\n");
			exit(1);
		}
		for (i = 0; i < config.max_uart_mode; i++) {
			if (i == RS232_MODE)
				strcpy(cur_mode, "RS232_GPIO_MODE");
			else if (i == RS485_2W_MODE)
				strcpy(cur_mode, "RS485_GPIO_MODE");
			else if (i == RS422_RS485_4W_MODE)
				strcpy(cur_mode, "RS422_GPIO_MODE");

			json_object_object_get_ex(jarray, cur_mode, &jitem);
			arraylen = json_object_array_length(jitem);

			for (j = 0 ; j < arraylen; j++) {
				jsubitem = json_object_array_get_idx(jitem, j);
				config.gpio_mode[i][j] =
					json_object_get_int(jsubitem);
			}
		}
	}
}

void check_device_node(char *device_node)
{
	FILE *fp;

	if (device_node && !device_node[0]) {
		printf("Device node was empty. Exit now.\n");
		usage();
		exit(0);
	}

	fp = fopen(device_node, "r");
	if (!fp) {
		printf("Open device node %s was failed! Exit now.\n",
			device_node);
		usage();
		exit(0);
	}
}

void check_uart_mode(int uart_mode)
{
	if (uart_mode < 0) {
		printf("UART mode %d was not existed! Exit now.\n", uart_mode);
		usage();
		exit(0);
	}
}

void show_uart_mode_message(int interface)
{
	switch (interface) {
	case RS232_MODE:
		printf("Now setting is RS232 interface.\n");
		break;
	case RS485_2W_MODE:
		printf("Now setting is RS485-2W interface.\n");
		break;
	case RS422_RS485_4W_MODE:
		printf("Now setting is RS422/RS485-4W interface.\n");
		break;
	default:
		printf("Unknown interface is set.\n");
	}
}

int main(int argc, char *argv[])
{
	int option;
	char optstring[] = "f:m:";

	config.set_uart_mode = -1;

	/* to check config file */
	load_config_file(CONFIG_NAME);

	if (strcmp(config.method, "DISABLE") == 0) {
		printf("mx-uart-ctl is disable. Exit.\n");
		return 0;
	}

	/* to check command line arguments */
	if (argc < 2) {
		usage();
		return 0;
	}

	while ((option = getopt(argc, argv, optstring)) != -1) {
		switch (option) {
		case 'f':
			snprintf(config.device_node,
				sizeof(config.device_node), "%s", optarg);
			check_device_node(config.device_node);
			config.ctl_mode = GET_MODE;
			break;
		case 'm':
			config.set_uart_mode = atoi(optarg);
			check_uart_mode(config.set_uart_mode);
			config.ctl_mode = SET_MODE;
			break;
		default:
			usage();
			return -1;
		}
	}

	/* to determine the control uart method */
	if (strcmp(config.method, "IOCTL") == 0) {
		exec_ioctl_cmd();
	} else if (strcmp(config.method, "GPIO") == 0) {
		gpio_init();
		exec_gpio_cmd();
	} else {
		printf("Error METHOD in %s\n", CONFIG_NAME);
		return -1;
	}

	return 0;
}
