#ifndef __INI_PARSE__
#define __INI_PARSE__

// debug message
#define logd(  dbg, x, y... ) do{ \
    if(dbg == 1 )\
    printf( x, ##y ); }while(0)

#define INI_FILENAME "/mnt/mtdblock1/ir8062.ini"

#define FRAME_NUMBER 			10
//#define CONNECTION_UNDEFINE		0
//#define CONNECTION_RJ45			1
//#define CONNECTION_RS485		2
//#define HTTP_POST_DATA_UNDEFINE	0
//#define HTTP_POST_FULL_DATA		1
//#define HTTP_POST_SIMPLE_DATA	2
#define XMAX	80
#define YMAX	62
#define CONNECTIVITY_TITLE "CONNECTIVITY"
#define FRAME_DISABLE		0
#define FRAME_ENABLE		1
#define FRAME_PARAMS_ERROR	2
#define INI_PARSE_DEBUG		0
#define ETHERNET_DEVICE_NAME		"eth0"
enum {
	CONNECTION_UNDEFINE = 0,
	RJ45,
	RS485,
	POST_DATA_UNDEFINE,
	POST_FULL_DATA,
	POST_SIMPLE_DATA
};
enum {
	DO1_ON = 0,
	DO2_ON,
	DO1_OFF,
	DO2_OFF
};

typedef struct {
	unsigned char conn_type; // RJ45, RS485
	unsigned char post_type; // full , simple
	unsigned char alarm;
	unsigned char dio_support;
	unsigned int over_alarm1;
	unsigned int over_alarm2;
	unsigned int under_alarm;
}system_param_t;

typedef struct {
	int status;
	int x;
	int y;
	int w;
	int h;
	int alarm_status;
	float over_temperature;
	float under_temperature;
}tFrame_t;

char* dio_command_get(char fun);
tFrame_t get_ini_frameinfo(int id);
void ir8062_params_print();
int ir8062_get_connectivity();
int parse_ini_file(const char* filename);
unsigned char get_rs485_cmd_len();
unsigned char get_ini_conn_type();
unsigned char get_ini_post_type();
unsigned char get_ini_alarm();
unsigned int get_ini_over_alarm1();
unsigned int get_ini_over_alarm2();
unsigned int get_ini_under_alarm();
#endif
