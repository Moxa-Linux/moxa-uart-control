AM_CFLAGS = -I$(top_srcdir)/include/
AM_CFLAGS += -Wall -Wextra -g
LDADD = $(top_builddir)/lib/libmx_uart_ctl.la -ljson-c -lmx_gpio_ctl
sbin_PROGRAMS = mx-uart-ctl
mx_uart_ctl_SOURCES = mx-uart-ctl.c
