#ifndef __RS485__
#define __RS485__

#define TX_BUFFER	128
#define RX_BUFFER	128

#define RS485_DEV "/dev/ttyS8" // rs485 port
//#define RS485_DEV "/dev/ttyS4" // uart port
//#define RS485_BAUDRATE 9600
typedef struct {
	int dev;
	unsigned int baudrate;
	char tx_buf[TX_BUFFER];
	char rx_buf[RX_BUFFER];
} rs485_t;


#define RS485_DEFAULT_ID	0xAA

// define rs485 command id
#define RS485_SET_DEVICE_ID		0x80
#define RS485_SET_DEVICE_MODE	0x81

#define RS485_SET_FRAME			0x70
#define RS485_SET_SQUARE1		0x71
#define RS485_SET_SQUARE2		0x72
#define RS485_SET_SQUARE3		0x73
#define RS485_SET_SQUARE4		0x74
#define RS485_SET_SQUARE5		0x75
#define RS485_SET_SQUARE6		0x76	
#define RS485_SET_SQUARE7		0x77
#define RS485_SET_SQUARE8		0x78
#define RS485_SET_SQUARE9		0x79

#define RS485_GET_ALARM_STATUS  0x82  //return alarm id(0~9) and max temperature length 20 bytes
#define RS485_GET_SQUARE_STAUTS	0x83 // return all max and min temperature, 20 bytes data
#define RS485_GET_FRAME_STATUS	0x84 // reverse command, 


#define RS485_SET_SERVER_IP		0x50
#define RS485_SET_SOCKET_START	0x51
#define RS485_SET_SCCKET_STOP	0x52
int rs485_init();
unsigned char get_square_alarm(unsigned char square);
unsigned char get_device_id();
unsigned char get_device_mode();
unsigned char get_server_command();
int send_sensor_info(unsigned char *data, int len, unsigned char cmd);
#endif