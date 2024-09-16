#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <time.h>
#include "../leds/pwm_message.h"
#include "led_control.h"

#define HEARTBIT_PERIOD	2000000 // 2 sec  
static clock_t heart_clk=0;
int msgid;
char buffer[16];
int led_send_msg(unsigned int msg) {
    struct message msg_data;
	int ret=0;	
    msg_data.msg_type = 1;
	msg_data.msg_text[0]=msg;
	if (msgsnd(msgid, &msg_data, sizeof(msg_data.msg_text), 0) == -1) {
        perror("msgsnd");
        ret = 1;
    }
    //printf("Data sent: %s", msg_data.msg_text);
    return ret;
}
void led_heartbit() {
	if ((clock()-heart_clk) > HEARTBIT_PERIOD) {
		printf("Send LED heartbit message\n");
		led_send_msg(MSG_HEART_BIT);
		heart_clk = clock();
	}
}
int led_msg_init() {
	msgid = msgget(MSG_KEY, 0666);
	if (msgid == -1) {
		perror("msgget");
		return 1;
		//exit(EXIT_FAILURE);
	}
	led_send_msg(MSG_LEDW_BREATHING);

	led_send_msg(MSG_LEDB_DISABLE);
	return 0;
}