#CC=arm-linux-gnueabihf-gcc
EXEC=mx-uart-ctl

CFLAGS += -I.

all: $(EXEC).c

$(EXEC).c:	$(EXEC).o
	$(CC) $(LDFLAGS) $(EXEC).o -o $(EXEC) -ljson-c
	$(STRIP) -s $(EXEC)

$(EXEC).o:
	$(CC) $(LDFLAGS) $(CFLAGS) -c $(EXEC).c

clean distclean:
	-rm -fr $(EXEC) $(EXEC).o
