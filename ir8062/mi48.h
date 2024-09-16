#ifndef __MI48_H__
#define __MI48_H__

typedef struct {
	unsigned int frame_cnt;
	unsigned int max;
	unsigned int min;
}mi48_header_t;

typedef struct {
	unsigned char alarm;
	unsigned char temperature;
}oseeing_config_t;

typedef struct {
	unsigned short int max;
	unsigned short int min;
}temperature_t;

typedef struct {
	unsigned char x1;
	unsigned char y1;
	unsigned char x2;
	unsigned char y2;
}coordinate_t;


//int mi48_scan(char *data);
int mi48_scan();
uint8_t *mi48_get_data();
int mi48_init();
int mi48_close();
unsigned int mi48_get_max_temperature();
unsigned int mi48_get_min_temperature();
#endif
