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

#ifndef _MOXA_UART_H
#define _MOXA_UART_H

#define MOXA_SET_OP_MODE                (0x400+66)
#define MOXA_GET_OP_MODE                (0x400+67)
#define UC_SET_SPECIAL_BAUD_RATE        (0x400+68)
#define UC_GET_SPECIAL_BAUD_RATE        (0x400+69)
#define MOXA_SET_SPECIAL_BAUD_RATE      (0x400+100)
#define MOXA_GET_SPECIAL_BAUD_RATE      (0x400+101)

enum uart_mode {
	UART_MODE_RS232 = 0,
	UART_MODE_RS485_2W = 1,
	UART_MODE_RS422_RS485_4W = 2
};

enum {
	MSP_PARITY_NONE,
	MSP_PARITY_ODD,
	MSP_PARITY_EVEN,
	MSP_PARITY_SPACE,
	MSP_PARITY_MARK
};

#ifdef __cplusplus
extern "C" {
#endif

extern int mx_uart_init(void);
extern int mx_uart_get_mode(int port, int *mode);
extern int mx_uart_set_mode(int port, int mode);

extern int mx_uart_open(int port);
extern int mx_uart_close(int port);
extern int mx_uart_read(int port, char *data, size_t count);
extern int mx_uart_write(int port, char *data, size_t count);
extern int mx_uart_set_baudrate(int port, int baudrate);
extern int mx_uart_get_baudrate(int port, int *baudrate);
extern int mx_uart_set_databits(int port, int bits);
extern int mx_uart_get_databits(int port, int *bits);
extern int mx_uart_set_stopbits(int port, int bits);
extern int mx_uart_get_stopbits(int port, int *bits);
extern int mx_uart_set_parity(int port, int parity);
extern int mx_uart_get_parity(int port, int *parity);

#ifdef __cplusplus
}
#endif

#endif /* _MOXA_UART_H */

