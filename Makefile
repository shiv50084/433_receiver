CC=gcc
CFLAGS=-I.
DEPS =

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

read_serial: read_serial.o 
	gcc -pthread -o read_serial read_serial.o -I.
