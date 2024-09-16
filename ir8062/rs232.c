/*
 * Copyright (c) 2014 Nuvoton technology corporation
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
 
#include     <stdio.h>
#include     <stdlib.h>
#include     <unistd.h> 
#include     <sys/types.h>
#include     <sys/stat.h> 
#include     <fcntl.h> 
#include     <termios.h>  
#include     <errno.h>
#include     <string.h>
#include 	<signal.h>
#include    <pthread.h>
#include <linux/serial.h>
#include "ini-parse.h"
#include "rs485.h"
#define RS485_DEV "/dev/ttyS4" // uart port
#define RS232_BAUDRATE 115200
// File name
// save the device id to replace default id
#define FILE_RS485_CUSTOM_ID	"/mnt/mtdblock1/oseeing1s-config/id" 
// socket server ip
#define FILE_SERVER_IP	"/ip" 
// set 1, start connect to socket server
#define FILE_SERVER_CONNECTION	"/connect" 
// bit 7 = 0 , full frame mode, =1 9 square mode,
// bit 6:0 , full frame alarm temperature
#define FILE_DEVICE_MODE	"/mnt/mtdblock1/oseeing1s-config/mode" 
// bit 7 = 1, enable alarm, bit 6:0 , alarm temperature
#define FILE_FRAME_ALARM	"/mnt/mtdblock1/oseeing1s-config/frame"
#define FILE_SQUARE1_ALARM	"/mnt/mtdblock1/oseeing1s-config/square1"
#define FILE_SQUARE2_ALARM	"/mnt/mtdblock1/oseeing1s-config/square2"
#define FILE_SQUARE3_ALARM	"/mnt/mtdblock1/oseeing1s-config/square3"
#define FILE_SQUARE4_ALARM	"/mnt/mtdblock1/oseeing1s-config/square4"
#define FILE_SQUARE5_ALARM	"/mnt/mtdblock1/oseeing1s-config/square5"
#define FILE_SQUARE6_ALARM	"/mnt/mtdblock1/oseeing1s-config/square6"
#define FILE_SQUARE7_ALARM	"/mnt/mtdblock1/oseeing1s-config/square7"
#define FILE_SQUARE8_ALARM	"/mnt/mtdblock1/oseeing1s-config/square8"
#define FILE_SQUARE9_ALARM	"/mnt/mtdblock1/oseeing1s-config/square9"

// end File name

unsigned char server_cmd=0;

#define BUFFER_SIZE	32
const unsigned char RESET_DEVICE_ID[6]={0xAA,0xFF,0x5A,0xA5,0x03,0x24};

pthread_t uart_tid;
int serial_port;
static unsigned char rs485_id = 0xAA;
static char buffer[BUFFER_SIZE];
static char rx[BUFFER_SIZE];
static int serial_init()
{
	struct termios tty;
	serial_port = open(RS485_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK); // 以读写方式打开串口设备
	if (serial_port < 0) {
		printf("Error opening serial port");
		return 1;
	}

	memset(&tty, 0, sizeof(tty));
	if (tcgetattr(serial_port, &tty) != 0) {
		printf("Error getting serial port attributes");
		close(serial_port);
		return 1;
	}

	tty.c_cflag &= ~CSTOPB; // 1 stop bit
	tty.c_cflag |= CS8; // 8 bit
	tty.c_cflag |= CREAD | CLOCAL; 
	cfsetospeed(&tty, RS232_BAUDRATE); // baudrate 115200
	printf("COM port = %s, baudrate = 115200\n", RS485_DEV);
	//cfsetospeed(&tty, 9600); // baudrate 115200
	//printf("COM port = %s, baudrate = 9600\n", RS485_DEV);
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;

	tty.c_cc[VMIN] = 0; // non-blocking
	tty.c_cc[VTIME] = 0; // non-blocking

    tcflush(serial_port, TCIFLUSH);
	tcsetattr(serial_port, TCSANOW, &tty);
	return 0;
}

static void sendcmd(char *cmd, ssize_t len){
	write(serial_port,cmd,len);
}
static void echo_rx_data(char *rx, ssize_t len) {
	memset(buffer, 0 , BUFFER_SIZE);
	memcpy(buffer, rx, len);
	printf("Echo RX Data len = %d\n",len);
	write(serial_port,buffer,len);
}
static ssize_t readdev(char *rx) {
	ssize_t rx_size=0;
	char *ip;
	memset(rx,0,BUFFER_SIZE);
	rx_size = read(serial_port, rx, BUFFER_SIZE);
	//if (rx_size<=0)
	//	return -1;
	//ip = &rx[3];
	//printf("ID(%x), cmd(%x), length(%d)\n", rx[0],rx[1],rx[2]);
	//printf("RX(%d) = %s\n", rx_size, ip);
	return rx_size;
}

static int write_square_alarm(unsigned char square, unsigned char data) {
	FILE *file=NULL;
	switch (square) {
		case 0 : 
			file = fopen(FILE_FRAME_ALARM,"wb");
			break;
		case 1:
			file = fopen(FILE_SQUARE1_ALARM,"wb");
			break;
		case 2:
			file = fopen(FILE_SQUARE2_ALARM,"wb");
			break;
		case 3:
			file = fopen(FILE_SQUARE3_ALARM,"wb");
			break;
		case 4:
			file = fopen(FILE_SQUARE4_ALARM,"wb");
			break;
		case 5:
			file = fopen(FILE_SQUARE5_ALARM,"wb");
			break;
		case 6:
			file = fopen(FILE_SQUARE6_ALARM,"wb");
			break;
		case 7:
			file = fopen(FILE_SQUARE7_ALARM,"wb");
			break;
		case 8:
			file = fopen(FILE_SQUARE8_ALARM,"wb");
			break;
		case 9:
			file = fopen(FILE_SQUARE9_ALARM,"wb");
			break;
		default:
			printf("Unknow square number %d\n", square);
			return -1;
			break;
	}
	if (file == NULL) {
		printf("Can't open SQUARE%d file\n", square);
		return -1;
	}
	size_t written = fwrite(&data, 1, 1, file);
	if (written != 1) {
		fclose(file);
		printf("Write RS485 ID(0x%x) faile\n", data);
		return -1;
	}
	fclose(file);
	return 0;
}

static int write_server_connect(unsigned char conn) {
	FILE *file = fopen(FILE_SERVER_CONNECTION,"wb");
	if (file == NULL) {
		printf("Can't open file %s\n", FILE_SERVER_CONNECTION);
		return -1;
	}
	size_t written = fwrite(&conn, 1, 1, file);
	if (written != 1) {
		fclose(file);
		printf("Write RS485 ID(0x%x) faile\n", conn);
		return -1;
	}
	fclose(file);
	return 0;
}

static unsigned char get_server_connect() {
	FILE *file=NULL;
	unsigned char conn;
	if (access(FILE_SERVER_CONNECTION, F_OK) != -1) {
		file = fopen(FILE_SERVER_CONNECTION,"rb");
		if (file == NULL) {
			printf("Can't open file %s\n", FILE_SERVER_CONNECTION);
			return 0;
		}
		size_t read = fread(&conn, 1, 1, file);
		if (read != 1) {
			fclose(file);
			printf("Read RS485 ID(%d) fail\n", conn);
			return 0;
		}
		fclose(file);
		return conn;
	} 
	else {
		printf("File %s not exist\n",FILE_SERVER_CONNECTION);
		return 0;
    }
}

static int write_server_ip(unsigned char *ip) {
	FILE *file = fopen(FILE_SERVER_IP,"wb");
	printf("IP = %s, len=%d\n", ip, strlen(ip));
	if (file == NULL) {
		printf("Can't open file %s\n", FILE_SERVER_IP);
		return -1;
	}
	size_t written = fwrite(ip, sizeof(unsigned char), strlen(ip), file);
	if (written != strlen(ip)) {
		fclose(file);
		printf("Write server IP %s faile\n", ip);
		return -1;
	}
	fclose(file);
	return 0;
}

static int get_server_ip(unsigned char *ip) {
	FILE *file=NULL;
	long filesize;
	if (access(FILE_SERVER_IP, F_OK) != -1) {
		file = fopen(FILE_SERVER_IP,"rb");
		if (file == NULL) {
			printf("Can't open file %s\n", FILE_SERVER_IP);
			return -1;
		}
		fseek(file,0,SEEK_END);
		filesize=ftell(file);
		rewind(file);
		if (filesize>16) { // MAC address max size is 16 byte
			fclose(file);
			printf("Wrong file size of ip=%d bytes\n", filesize);
			return -1;
		}
		memset(ip,0,17);
		size_t read = fread(ip, 1, filesize, file);
		if (read != filesize) {
			fclose(file);
			printf("Read ip(%s) fail\n", ip);
			return -1;
		}
		fclose(file);
		return 0;
	} 
	else {
		printf("File %s not exist\n",FILE_SERVER_IP);
		return -1;
    }
}

static int write_device_id(unsigned char id) {
	FILE *file = fopen(FILE_RS485_CUSTOM_ID,"wb");
	if (file == NULL) {
		printf("Can't open file %s\n", FILE_RS485_CUSTOM_ID);
		return -1;
	}
	size_t written = fwrite(&id, sizeof(unsigned char), 1, file);
	if (written != 1) {
		fclose(file);
		printf("Write RS485 ID(0x%x) faile\n", id);
		return -1;
	}
	fclose(file);
	printf("Current ID (%d) change to New ID (%d)\n", rs485_id, id);
	rs485_id = id;
	return 0;
}

static int write_device_mode(unsigned char mode) {
	FILE *file = fopen(FILE_DEVICE_MODE,"wb");
	if (file == NULL) {
		printf("Can't open file %s\n", FILE_DEVICE_MODE);
		return -1;
	}
	size_t written = fwrite(&mode, sizeof(unsigned char), 1, file);
	if (written != 1) {
		fclose(file);
		printf("Write RS485 ID(0x%x) faile\n", mode);
		return -1;
	}
	fclose(file);
	return 0;
}

static int is_reset_id_cmd(char *rx, ssize_t len) {
	// Check reset rs485 id

	if (len == 6) {
		if (memcmp(rx,RESET_DEVICE_ID,6)==0) {
			if (access(FILE_RS485_CUSTOM_ID, F_OK) < 0) {
				printf("No custom ID define...\n");
				return 1;
			}
			if (remove(FILE_RS485_CUSTOM_ID) == 0) {
				printf("Reset RS485 ID as 0xAA\n");
				rs485_id = RS485_DEFAULT_ID;
				return 1;
			}
		}
		else {
			printf("Reset RS485 ID fail...\n");
			return -1;
		}
	}
	else
		return 0;
}
static int check_command_format(unsigned char id, unsigned char cmd, unsigned cmdlen, ssize_t len) {
	if (id != rs485_id) {
		printf("Unmatch RS485 ID(0x%x) with %x\n", rs485_id, id);
		return -1;
	}
	if ( (cmdlen+3) != (len) ) {
		printf("rx data lose, length(%d) not match data length(%d)\n", len, cmdlen);
		return -1;
	}
	if ( (cmd>0x90) || (cmd<0x50) ) {
		printf("Command(0x%x) out of define\n", cmd);
		return -1;
	}
	return 0;
}
static int rx_data_parse(char *rx, ssize_t len) {
	int ret = 0;
	unsigned char *data;
	unsigned char id = rx[0];
	unsigned char cmd = rx[1];
	unsigned char cmdlen;
	ret = is_reset_id_cmd(rx,len) ; 
	if (ret == 1) {
		echo_rx_data(rx,len);
		return 1;
	}
	else if (ret <0) return -1;
	
	cmdlen=rx[2];
	if ( check_command_format (id, cmd, cmdlen, len) < 0 ) {
		printf("Incorrect command format..\n");
		return -1;
	}
	switch (cmd) {
		case RS485_SET_DEVICE_ID:
			if (rs485_id == rx[3]) {
				printf("New ID (%d) == evice ID (%d) \n", rx[3], rs485_id);
				usleep(50000);
				echo_rx_data(rx,len);
				return 1;
			}
			if (write_device_id(rx[3])==0) {
				echo_rx_data(rx,len);
				rs485_id = rx[3];
			}
			else {
				printf("RS485_SET_DEVICE_ID fail...\n");
				return -1;
			}
			break;
		case RS485_SET_DEVICE_MODE:
			if (write_device_mode(rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_DEVICE_MODE fail...\n");
				return -1;
			}			
			break;
		case RS485_SET_FRAME:
			if (write_square_alarm(0,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE1 fail...\n");
				return -1;
			}
			break;
		case RS485_SET_SQUARE1:
			if (write_square_alarm(1,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE1 fail...\n");
				return -1;
			}			
			break;
		case RS485_SET_SQUARE2:
			if (write_square_alarm(2,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE2 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE3:
			if (write_square_alarm(3,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE3 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE4:
			if (write_square_alarm(4,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE4 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE5:
			if (write_square_alarm(5,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE5 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE6:
			if (write_square_alarm(6,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE6 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE7:
			if (write_square_alarm(7,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE7 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE8:
			if (write_square_alarm(8,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE8 fail...\n");
				return -1;
			}			break;
			break;
		case RS485_SET_SQUARE9:
			if (write_square_alarm(9,rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SQUARE9 fail...\n");
				return -1;
			}
			break;
		case RS485_GET_ALARM_STATUS :
			printf("Get alarm command\n");
			server_cmd = RS485_GET_ALARM_STATUS;
			break;
		case RS485_GET_FRAME_STATUS:
			server_cmd = RS485_GET_FRAME_STATUS;
			break;
		case RS485_GET_SQUARE_STAUTS:
			printf("Get Square command\n");
			server_cmd = RS485_GET_SQUARE_STAUTS;
			break;
		case RS485_SET_SERVER_IP:
			if (write_server_ip(&rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SERVER_IP fail...\n");
				return -1;
			}
			break;
		case RS485_SET_SOCKET_START:
			if (write_server_connect(rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SOCKET_START fail...\n");
				return -1;
			}
			break;
		case RS485_SET_SCCKET_STOP:
			if (write_server_connect(rx[3])==0)
				echo_rx_data(rx,len);
			else {
				printf("RS485_SET_SCCKET_STOP fail...\n");
				return -1;
			}
			break;
		default:
			printf("Unknow command id 0x%x\n", cmd);
			break;
	}
	
}

// export function

unsigned char get_square_alarm(unsigned char square) {
	FILE *file=NULL;
	unsigned char data;
	switch (square) {
		case 0:
			if (access(FILE_FRAME_ALARM, F_OK) != -1)
				file = fopen(FILE_FRAME_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 1:
			if (access(FILE_SQUARE1_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE1_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 2:
			if (access(FILE_SQUARE2_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE2_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 3:
			if (access(FILE_SQUARE3_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE3_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 4:
			if (access(FILE_SQUARE4_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE4_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 5:
			if (access(FILE_SQUARE5_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE5_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 6:
			if (access(FILE_SQUARE6_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE6_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 7:
			if (access(FILE_SQUARE7_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE7_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 8:
			if (access(FILE_SQUARE8_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE8_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		case 9:
			if (access(FILE_SQUARE9_ALARM, F_OK) != -1)
				file = fopen(FILE_SQUARE9_ALARM,"rb");
			else {
				printf("SQUARE %d file not exist...\n", square);
				return -1;
			}				
			break;
		default:
			printf("Unknow square number %d\n", square);
			break;
	}
	if (file == NULL) {
		printf("Can't open file %s\n", FILE_SERVER_CONNECTION);
		return 0;
	}
	size_t read = fread(&data, 1, 1, file);
	if (read != 1) {
		fclose(file);
		printf("Read(%d) RS485 data(%d) fail\n", read, data);
		return 0;
	}
	fclose(file);
	return data;
}

unsigned char get_device_id() {
	FILE *file=NULL;
	unsigned char id;
	if (access(FILE_RS485_CUSTOM_ID, F_OK) != -1) {
		file = fopen(FILE_RS485_CUSTOM_ID,"rb");
		if (file == NULL) {
			printf("Can't open file %s\n", FILE_RS485_CUSTOM_ID);
			return RS485_DEFAULT_ID;
		}
		size_t read = fread(&id, 1, 1, file);
		if (read != 1) {
			fclose(file);
			printf("Read RS485 ID(%d) fail\n", id);
			return RS485_DEFAULT_ID;
		}
		fclose(file);
		return id;
	} 
	else {
		printf("Use default RS485 ID 0xAA\n");
		return RS485_DEFAULT_ID;
    }
}

unsigned char get_device_mode() {
	FILE *file=NULL;
	unsigned char mode;
	if (access(FILE_DEVICE_MODE, F_OK) != -1) {
		file = fopen(FILE_DEVICE_MODE,"rb");
		if (file == NULL) {
			printf("Can't open file %s\n", FILE_DEVICE_MODE);
			return 0;
		}
		size_t read = fread(&mode, 1, 1, file);
		if (read != 1) {
			fclose(file);
			printf("Read RS485 Device mode(%d) fail\n", mode);
			return 0;
		}
		fclose(file);
		return mode;
	} 
	else {
		printf("File %s not exist\n",FILE_DEVICE_MODE);
		return 0;
    }	
}

unsigned char get_server_command() {
	unsigned char ret = server_cmd;
	server_cmd = 0;
	return ret;
}
int send_sensor_info(unsigned char *data, int len, unsigned char cmd) {
	int i;
	if (len <=0 ) {
		printf("Not assisgned return data\n");
		return -1;
	}
	
	memset(buffer, 0 , BUFFER_SIZE);
	buffer[0]=rs485_id;
	buffer[1]=cmd;
	buffer[2]=len;
	if (len != 0) {
		memcpy(&buffer[3], data, len);
	}
	write(serial_port,buffer,len+3);
	printf("Send info(%d) = ", len+3);
	for (i=0 ; i<len+3; i++) 
		printf("0x%x ", buffer[i]);
	printf("\n");
	return 0;
}

// end export function
void *uart_thread(void *arg) {
	ssize_t rx_size = 0;
	int i;
    while (1) {

		rx_size=readdev(rx);
		if (rx_size > 3)
			rx_data_parse(rx,rx_size);
    }
    close(serial_port);
	printf("EXIT uart\n");

    return;
}
void curl_thread_destory() {
    pthread_cancel(uart_tid);
    pthread_join(uart_tid, NULL);  
}	
int uart_thread_create() {
    if (pthread_create(&uart_tid, NULL, uart_thread, NULL) != 0) {
        perror("uart pthread_create");
        return -1;
    }
    return 0;
}
int rs485_init() {
	serial_init();
	uart_thread_create();
}
