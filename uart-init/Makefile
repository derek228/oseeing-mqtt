.SUFFIXES : .x .o .c .s

#STRIP := aarch64-poky-linux-strip

CC := arm-linux-gcc
#CC := gcc

TARGET = uart


SRCS := uart.c



all:

	$(CC) $(SRCS) -o $(TARGET)




clean:
	rm -f *.o
	rm -f *.x
	rm -f *.flat
	rm -f *.map
	rm -f temp
	rm -f *.img
	rm -f $(TARGET)
	rm -f *.gdb
	

