#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#define MSG_SIZE 128
#define MSG_KEY  ftok("./testpwm", 'a')  // 使用 ftok 创建键值，'/tmp' 可以替换成其他路径或文件名
enum {
MSG_NONE = 0,
MSG_LEDR_ENABLE, // 1
MSG_LEDR_DISABLE, 
MSG_LEDR_BLANKING,
MSG_LEDG_ENABLE, 
MSG_LEDG_DISABLE, 
MSG_LEDG_BLANKING, // 5
MSG_LEDB_ENABLE, // 
MSG_LEDB_DISABLE, // 
MSG_LEDB_BLANKING,
MSG_LEDY_ENABLE,		// 7 , stop breath
MSG_LEDY_DISABLE, // 10
MSG_LEDY_BLANKING,
MSG_LEDW_ENABLE, // 12
MSG_LEDW_DISABLE, 
MSG_LEDW_BREATHING, // 11
// breath cycle time in sec
MSG_CYCLE_TIME_3, // 12
MSG_CYCLE_TIME_4, // 13
MSG_CYCLE_TIME_5, // 14
MSG_CYCLE_TIME_6, // 15
MSG_CYCLE_TIME_7, // 16
MSG_HEART_BIT,
MSG_EXIT	//
};

/*
#define CMD_LED_GPIO_SET	0x01
#define CMD_LED_PWM_SET		0x02

struct led_msg {
	char cmd;
	char color;
	char enable;
	char blanking;
	char blanking_freq; // sec
	char breathing;
	char breathing_freq;
};
struct pwm_msg {
	char cmd;
	char breathing;
	char breathing_freq; // sec
	char enable;	
};
*/
struct message {
    long msg_type;
    char msg_text[MSG_SIZE];
};
#endif /* MESSAGE_QUEUE_H */

