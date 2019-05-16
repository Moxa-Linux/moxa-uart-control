/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Name:
 *	MOXA UART Control Utility
 *
 * Description:
 *	Utility for getting or setting UART mode.
 *
 * Authors:
 *	2005	Victor Yu	<victor.yu@moxa.com>
 *	2017	Elvis CW Yao	<ElvisCW.Yao@moxa.com>
 *	2018	Ken CJ Chou	<KenCJ.Chou@moxa.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <mx_uart.h>

#define UNSET -1

enum action_type {
	GET_MODE = 0,
	SET_MODE = 1
};

struct action_struct {
	int type;
	int port;
	int mode;
};

void usage(FILE *fp)
{
	fprintf(fp, "Usage:\n");
	fprintf(fp, "	mx-uart-ctl -p <#port_number> -m <#uart_mode>\n\n");
	fprintf(fp, "OPTIONS:\n");
	fprintf(fp, "	-p <#port_number>\n");
	fprintf(fp, "		Set target port: [0..]\n");
	fprintf(fp, "	-m <#uart_mode>\n");
	fprintf(fp, "		Set target port to uart_mode\n");
	fprintf(fp, "		0 --> set to RS232 mode\n");
	fprintf(fp, "		1 --> set to RS485-2W mode\n");
	fprintf(fp, "		2 --> set to RS422/RS485-4W mode\n");
	fprintf(fp, "\n");
	fprintf(fp, "Example:\n");
	fprintf(fp, "	Get mode from port 0\n");
	fprintf(fp, "	# mx-uart-ctl -p 0\n");
	fprintf(fp, "\n");
	fprintf(fp, "	Set port 1 to mode RS232\n");
	fprintf(fp, "	# mx-uart-ctl -p 1 -m 0\n");
}

int my_atoi(const char *nptr, int *number)
{
	int tmp;

	tmp = atoi(nptr);
	if (tmp != 0) {
		*number = tmp;
		return 0;
	} else {
		if (!strcmp(nptr, "0")) {
			*number = 0;
			return 0;
		}
	}
	return -1;
}

void show_uart_mode_message(int mode)
{
	switch (mode) {
	case UART_MODE_RS232:
		printf("Current uart mode is RS232 interface.\n");
		break;
	case UART_MODE_RS485_2W:
		printf("Current uart mode is RS485-2W interface.\n");
		break;
	case UART_MODE_RS422_RS485_4W:
		printf("Current uart mode is RS422/RS485-4W interface.\n");
		break;
	default:
		printf("Unknown interface is set.\n");
		break;
	}
}

void do_action(struct action_struct action)
{
	int ret;

	switch (action.type) {
	case GET_MODE:
		ret = mx_uart_get_mode(action.port, &action.mode);
		if (ret < 0) {
			fprintf(stderr, "Get UART mode failed\n");
			exit(1);
		}
		break;
	case SET_MODE:
		if (mx_uart_set_mode(action.port, action.mode) < 0) {
			fprintf(stderr, "Set UART mode failed\n");
			exit(1);
		}
		printf("Set OK.\n");

		if (mx_uart_get_mode(action.port, &action.mode) < 0) {
			fprintf(stderr, "Get UART mode failed\n");
			exit(1);
		}
		break;
	}

	show_uart_mode_message(action.mode);
}

int main(int argc, char *argv[])
{
	struct action_struct action = {
		.type = GET_MODE,
		.port = UNSET,
		.mode = UNSET
	};
	int c;

	while (1) {
		c = getopt(argc, argv, "hp:m:");
		if (c == -1)
			break;

		switch (c) {
		case 'h':
			usage(stdout);
			exit(0);
		case 'p':
			if (my_atoi(optarg, &action.port) != 0) {
				fprintf(stderr, "%s is not a number\n", optarg);
				exit(1);
			}
			break;
		case 'm':
			action.type = SET_MODE;
			if (my_atoi(optarg, &action.mode) != 0) {
				fprintf(stderr, "%s is not a number\n", optarg);
				exit(1);
			}
			break;
		default:
			usage(stderr);
			exit(99);
		}
	}

	if (optind < argc) {
		fprintf(stderr, "non-option arguments found: ");
		while (optind < argc)
			fprintf(stderr, "%s ", argv[optind++]);
		fprintf(stderr, "\n");

		usage(stderr);
		exit(99);
	}

	if (action.port == UNSET) {
		fprintf(stderr, "port number is unset\n");
		usage(stderr);
		exit(99);
	}

	if (mx_uart_init() < 0) {
		fprintf(stderr, "Initialize Moxa uart control library failed\n");
		exit(1);
	}

	do_action(action);

	exit(0);
}
