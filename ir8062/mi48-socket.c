#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <getopt.h>
#include <linux/types.h>
#include <time.h>
#include <arpa/inet.h>
#include "spidev.h"
#include "ini-parse.h"
#include "mi48.h"
#include "rs485.h"
#define NEW_THERMAL_SCAN 1
#define SERVER_IP "192.168.100.70"  // Server IP address (fix ip )
#define SERVER_PORT 8080
int sockfd=-1;
struct sockaddr_in server_addr;
int socket_connected=-1;

// Thermal sensor hardware signal setting
#define CAP_SIG_ID          0x0a 
#define CMD_SIG_TASK_REG    _IOW(CAP_SIG_ID, 0, int32_t*)
#define CAP_MAX_LINE                32
#define CAP_INFO_LEN_UNIT_BYTE      12
#define CAP_MAX_LEN                 (CAP_MAX_LINE * CAP_INFO_LEN_UNIT_BYTE)
#define GPIO_INFO_SIZE_INT          3
#define CAP_MAX_LEN_INT             (CAP_MAX_LEN/4)
//gpio
int state_change = 0;
// SPI interface config
static const char *device = "/dev/spidev1.0";
//static const char *device = "/dev/spidev0.1";
//static const char *device = "/dev/spidev1.1";
static uint32_t mode=0;
static uint8_t bits = 8;
static uint32_t speed = 4000000; //500000    4000000
static uint16_t delay=0;
static int fd_spi;
static int fd_capture;
static uint8_t *tx;
static uint8_t *rx;
static int size;
static uint8_t mi48_data[9920]={0};
static unsigned short temp_kelvin[62][80]={0};
//static uint8_t data[9920]={0};

oseeing_config_t oseeing_config[10] = {0};
temperature_t temperature[10] = {0};
coordinate_t area[9]= { { 1, 1,26,20},
						{27, 1,52,20},
						{53, 1,78,20},
						{ 1,21,26,40},
						{27,21,52,40},
						{53,21,78,40},
						{ 1,41,26,60},
						{27,41,52,60},
						{53,41,78,60}
};

//static char mi48_header_raw[160]={0};
static mi48_header_t mi48_header;
static int mi48_debug=0;
static void mi48_log_print() {
	const char *filename = "/mnt/mtdblock1/mi48";
	if (access(filename, F_OK) != -1) {
		mi48_debug=1;
	} else {
		mi48_debug=0;
    }
}

void sig_event_handler(int sig_id, siginfo_t *sig_info, void *unused)
{
	if ( sig_id == CAP_SIG_ID ) {
		state_change = 1;
	}
}


#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void transfer(int fd, uint8_t const *tx, uint8_t const *rx, size_t len)
{
	int i;
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
	};
	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
}

static int memory_print() {
    unsigned long long free_memory;
    FILE *fp = fopen("/proc/meminfo", "r");
    if(fp == NULL) {
        printf("Error: Failed to open /proc/meminfo\n");
        return 1;
    }
    char line[128];
    while(fgets(line, sizeof(line), fp)) {
        if(sscanf(line, "MemFree: %llu kB", &free_memory) == 1) {
            free_memory *= 1024;
            break;
        }
    }
    fclose(fp);
    logd(mi48_debug,"Free memory: %llu bytes\n", free_memory);
    return 0;
}

static int ir8062_hwinit()
{
	//SPI
	int ret = 0;
	// GPIO
	struct sigaction act;
	int i;

	sigemptyset(&act.sa_mask);
	act.sa_flags = (SA_SIGINFO | SA_RESTART);
	act.sa_sigaction = sig_event_handler;
	sigaction(CAP_SIG_ID, &act, NULL);

	printf("signal handler= %d, spi device=%s\n", CAP_SIG_ID,device);

	fd_capture = open("/dev/gpio_cap", O_RDWR);
	if(fd_capture < 0) {
		printf("Open device fail\n");
		return -1;
	}

	fd_spi = open(device, O_RDWR);
	if (fd_spi < 0)
		pabort("can't open device");
	/*
	 * spi mode
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
	ret = ioctl(fd_spi, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	/*
	 * bits per word
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
	ret = ioctl(fd_spi, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
	/*
	 * max speed hz
	 */
	ret = ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
	ret = ioctl(fd_spi, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	
	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	size = 160;
	tx = malloc(size);
	rx = malloc(size);
	for (i=0; i<size; i++)
	{
		tx[i] = 0;//i;
		//printf("%x ", tx[i] );
	}
	printf("Reset MI48 \n");
	system("./i2cset -f -y 1 0x40 0 1");
	printf("MI48 reset done\n");
	system("./i2cset -f -y 1 0x40 0xB4 0x03");
	system("./i2cset -f -y 1 0x40 0xD0 0x02");
	system("./i2cset -f -y 1 0x40 0xD0 0x03");

	system("./i2cset -f -y 1 0x40 0xB1 0x03");
	system("./i2cget -f -y 1 0x40 0xB2");
	system("./i2cget -f -y 1 0x40 0xB3");
	system("./i2cget -f -y 1 0x40 0xE0");
	system("./i2cget -f -y 1 0x40 0xE1");
	system("./i2cget -f -y 1 0x40 0xE2");
	system("./i2cget -f -y 1 0x40 0xE3");
	system("./i2cget -f -y 1 0x40 0xE4");
	system("./i2cget -f -y 1 0x40 0xE5");
	return ret;
}

static void mi48_header_parse(uint8_t *mi48_header_raw) 
{
	mi48_header.frame_cnt = (mi48_header_raw[0]<<8) | mi48_header_raw[1];
	mi48_header.max = (mi48_header_raw[10]<<8) | mi48_header_raw[11];
	mi48_header.min = (mi48_header_raw[12]<<8) | mi48_header_raw[13];
	logd(mi48_debug,"frame count = %d, max=%d, min=%d\n",mi48_header.frame_cnt,mi48_header.max,mi48_header.min);
/*
	switch (index) {
		case 0: // Frame count
		case 1:
			logd(sensor_debug,"Frame counter : %02X",buf[index]);
			logd(sensor_debug,"%02X\n",buf[index]);
			break;
		case 2:
		case 3:
			logd(sensor_debug,"SenXor VDD : %02X",buf[index]);
			logd(sensor_debug,"%02X\n",buf[index]);
			break;
		case 4:
		case 5:
			logd(sensor_debug,"Die Temperature : %02X",buf[index]);
			logd(sensor_debug,"%02X\n",buf[index]);
			break;
		default :
			break;
	}
*/
}
#define FILE_RS485_CUSTOM_ID	"/mnt/mtdblock1/oseeing1s-config/id" 
// socket server ip
#define FILE_SERVER_IP	"/ip" 
// set 1, start connect to socket server
#define FILE_SERVER_CONNECTION	"/connect" 
// bit 7 = 0 , full frame mode, =1 9 square mode,
// bit 6:0 , full frame alarm temperature
#define FILE_DEVICE_MODE	"/mnt/mtdblock1/oseeing1s-config/mode" 
// bit 7 = 1, enable alarm, bit 6:0 , alarm temperature
#define FILE_SQUARE1_ALARM	"/mnt/mtdblock1/oseeing1s-config/square1"
#define FILE_SQUARE2_ALARM	"/mnt/mtdblock1/oseeing1s-config/square2"
#define FILE_SQUARE3_ALARM	"/mnt/mtdblock1/oseeing1s-config/square3"
#define FILE_SQUARE4_ALARM	"/mnt/mtdblock1/oseeing1s-config/square4"
#define FILE_SQUARE5_ALARM	"/mnt/mtdblock1/oseeing1s-config/square5"
#define FILE_SQUARE6_ALARM	"/mnt/mtdblock1/oseeing1s-config/square6"
#define FILE_SQUARE7_ALARM	"/mnt/mtdblock1/oseeing1s-config/square7"
#define FILE_SQUARE8_ALARM	"/mnt/mtdblock1/oseeing1s-config/square8"
#define FILE_SQUARE9_ALARM	"/mnt/mtdblock1/oseeing1s-config/square9"
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
		//printf("File %s not exist\n",FILE_SERVER_CONNECTION);
		return 0;
    }
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
int is_socket_connection() {
	// check config
	static unsigned char ipaddr[17]={0};
	if (get_server_connect()==0) {
		socket_connected = -1;
		if (sockfd >=0 )
			close(sockfd);
		sockfd = -1;
		
		return -1;
	}
	else {
		if (get_server_ip(ipaddr)<0)
			return -1;
	}
    // create socket
	if (sockfd <0) {
		printf("Socket disconnect, try to init...\n");
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(SERVER_PORT);
    // 将 IP 地址转换为二进制格式并存储在 server_addr 结构中
//		if (inet_pton(AF_INET, SERVER_IP, &server_addr.sin_addr) <= 0) {
		if (inet_pton(AF_INET, ipaddr, &server_addr.sin_addr) <= 0) {
			perror("Invalid address/ Address not supported");
			return -1;
		}

	    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
	        perror("Socket creation failed");
			return -1;
	    }
	}
	if (socket_connected < 0) {
	    // 连接到接收端
		socket_connected=connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
   		if (socket_connected < 0) {
	        perror("Connection Failed");
	        return -1;
	    }
		return 1;
	}
	return 1;
}

int oseeing_config_update() {
	unsigned char data, alarm, temp,i;
	for (i=0;i<10;i++) {
		data = get_square_alarm(i);
		if (data < 0) {
			printf("Can't get square(%x) data=%x\n", i,data);
			data = 0;
		}
		oseeing_config[i].alarm = ( data & 0x80 ) >> 7;
		oseeing_config[i].temperature = data & 0x7f ;
		printf("id(%d) : alarm = %d, temp = %d\n", i, oseeing_config[i].alarm,oseeing_config[i].temperature);
	}
	return 0;
}

int send_alarm_status(unsigned char cmd) {
	int i,j,len=0;
	unsigned char buf[20];
	printf("%s\n",__FUNCTION__);	
	for (i=0;i<10;i++) {
		if (oseeing_config[i].alarm) {
			if (oseeing_config[i].temperature < temperature[i].max) {
				buf[len] = i;
				buf[len+1] = temperature[i].max;
				len += 2;
				printf("len = %d ,ID(%d) temperature(%d) > alarm temperature(%d)\n", len, i, temperature[i].max, oseeing_config[i].temperature);
			}
			else 
				printf("ID(%d) temperature(%d) < alarm temperature(%d)\n", i, temperature[i].max, oseeing_config[i].temperature);
		}
		else {
			printf("ID(%d) alarm disable...\n",i);
		}
	}
	if (len ==0 ) {
		printf("No temperature alarm...\n");
		buf[0]=0;
	}
	send_sensor_info(buf,len,cmd);
}
// reverse
int send_frame_status(unsigned char cmd) {
	printf("%s\n",__FUNCTION__);

}
int send_square_status(unsigned char cmd) {
	unsigned char buf[20]={0};
	int i;
	for (i=0; i<10; i++) {
		buf[i*2]=temperature[i].max;
		buf[i*2+1]=temperature[i].min;
	}
	send_sensor_info(buf, 20, cmd);
	printf("%s\n",__FUNCTION__);
	
}

void mi48_raw_to_kelvin() {
	int i,j;
    for (i = 0; i < 62; i++) {
        for (j = 0; j < 80; j++) {
            temp_kelvin[i][j] = (mi48_data[(i * 80 + j) * 2] << 8) | mi48_data[(i * 80 + j) * 2 + 1];
        }
    }
	printf("temperature[30][30]= %d\n",temp_kelvin[30][30]);
}

int temperature_analysis(unsigned char cmd) {
	int i,j,k;
	unsigned short int max, min;
	oseeing_config_update();
	mi48_raw_to_kelvin();
	temperature[0].max = (mi48_header.max-2735) / 10;
	temperature[0].min = (mi48_header.min-2735) / 10;
	printf("Frame max temperature = %d, min temperature = %d\n", temperature[0].max,temperature[0].min);
	for (i=1; i<10; i++) {
		min=0xFFFF;
		max=0;
		for (j=area[i-1].y1; j<area[i-1].y2; j++) {
			for (k=area[i-1].x1; k<area[i-1].x2; k++) {
				if (temp_kelvin[j][k] > max)
					max = (temp_kelvin[j][k]);
					//max = (temp_kelvin[j][k]-2735)/10;
				if (temp_kelvin[j][k] < min)
					min = (temp_kelvin[j][k]);
					//min = (temp_kelvin[j][k]-2735)/10;
			}
			temperature[i].max = (max-2735)/10;
			temperature[i].min = (min-2735)/10;
		}
		printf("area[%d] x1(%d),y1(%d),x2(%d),y2(%d) : max=%d, min=%d\n", i, area[i-1].x1,area[i-1].y1,area[i-1].x2,area[i-1].y2,temperature[i].max,temperature[i].min);
	}
	switch (cmd) {
		case RS485_GET_ALARM_STATUS :
			send_alarm_status(cmd);
			break ; 
		case RS485_GET_SQUARE_STAUTS:
			send_square_status(cmd);
			break;
		case RS485_GET_FRAME_STATUS:
			send_frame_status(cmd);
			break;
		default : 
			printf("Unknow server request (0x%x)\n", cmd);
			return -1;
	}

}

//int mi48_scan(char *data) {
int mi48_scan() {
	int spi_count = 0;
	unsigned char cmd;
	mi48_log_print();
	size = 160;
	if (state_change == 1) 
	{
		//led_send_msg(MSG_HEART_BIT);
		state_change = 0;
		transfer(fd_spi,tx,rx,size);
		mi48_header_parse(rx);
		for (spi_count = 0; spi_count<62;spi_count++) {
			transfer(fd_spi,tx,rx,size);
			memcpy(&mi48_data[spi_count*160],rx,size);
		}
// Try to send mi48_data to socket
		if (is_socket_connection() > 0) {
			ssize_t sent_bytes = send(sockfd, mi48_data, sizeof(mi48_data), MSG_NOSIGNAL);
//			printf("sent_bytes = %d\n", sent_bytes);
			if (sent_bytes < 0) {
				perror("Failed to send mi48_data");
				socket_connected = -1;
				close(sockfd);
				sockfd = -1;
			}
		}
		// socket end
// Check modbus server request 
		cmd = get_server_command();
		if (cmd)
			temperature_analysis(cmd);
	}

	return 0; // fail, spi not ready
}
unsigned int mi48_get_max_temperature() {
	return mi48_header.max;
}

unsigned int mi48_get_min_temperature() {
	return mi48_header.min;
}

uint8_t *mi48_get_data() {
	return mi48_data;
}
int mi48_close()
{
	free(rx);
	free(tx);
	close(fd_spi);
	close(fd_capture);
	return 0;	
}
int socket_close() {

}

int mi48_init()
{
	int ret=0;
	ret=ir8062_hwinit();
	oseeing_config_update();
	if (ret < 0) 
		printf("ERROR : MI48 initial failure\n");
	return ret;
}
