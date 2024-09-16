.SUFFIXES : .x .o .c .s

CC := arm-linux-gcc
#STRIP := arm-linux-strip
TARGET = oseeing1s

OBJS= main.o rs485.o led_control.o ethernet.o mi48-socket.o cJSON.o mqtt.o
LIBS = -L/home/user/NUC970_Buildroot-master/output/build/paho-mqtt-c-v1.1.0/src/ -lpaho-mqtt3c -lcurl -lm -ljpeg #-lssl -lcrypto #-ljson-c -lnsl 

INCLUDES = -I/home/user/NUC970_Buildroot-master/output/build/paho-mqtt-c-v1.1.0/src/
CFLAGS = -Wall $(INCLUDES)

$(TARGET) : $(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(LDFLAGS) $(CFLAGS) $(LIBS) 

clean:
	rm -r *.o

