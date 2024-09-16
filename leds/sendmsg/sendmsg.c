#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include "../pwm_message.h"

int main(int argc, char **argv) {
    int msgid;
    struct message msg_data;
    char buffer[MSG_SIZE];
	int ret=0;
    // 创建或获取消息队列
/*    msgid = msgget(MSG_KEY, 0666 | IPC_CREAT);
    if (msgid == -1) {
        perror("msgget");
        exit(EXIT_FAILURE);
    }
*/
	// 获取相同的消息队列
	msgid = msgget(MSG_KEY, 0666);
	if (msgid == -1) {
		perror("msgget");
		exit(EXIT_FAILURE);
	}

    printf("Enter message: ");
    fgets(buffer, MSG_SIZE, stdin);
	// red
	if (strstr(buffer,"re") != NULL)
		msg_data.msg_text[0]=MSG_LEDR_ENABLE;
	else if (strstr(buffer,"rd") != NULL)
		msg_data.msg_text[0]=MSG_LEDR_DISABLE;
	else if (strstr(buffer,"rb") != NULL)
		msg_data.msg_text[0]=MSG_LEDR_BLANKING;
	// green
	else if (strstr(buffer,"ge") != NULL)
		msg_data.msg_text[0]=MSG_LEDG_ENABLE;
	else if (strstr(buffer,"gd") != NULL)
		msg_data.msg_text[0]=MSG_LEDG_DISABLE;
	else if (strstr(buffer,"gb") != NULL)
		msg_data.msg_text[0]=MSG_LEDG_BLANKING;
	// blue
	else if (strstr(buffer,"be") != NULL)
		msg_data.msg_text[0]=MSG_LEDB_ENABLE;
	else if (strstr(buffer,"bd") != NULL)
		msg_data.msg_text[0]=MSG_LEDB_DISABLE;
	else if (strstr(buffer,"bb") != NULL)
		msg_data.msg_text[0]=MSG_LEDB_BLANKING;
	// yellow
	else if (strstr(buffer,"ye") != NULL)
		msg_data.msg_text[0]=MSG_LEDY_ENABLE;
	else if (strstr(buffer,"yd") != NULL)
		msg_data.msg_text[0]=MSG_LEDY_DISABLE;
	else if (strstr(buffer,"yb") != NULL)
		msg_data.msg_text[0]=MSG_LEDY_BLANKING;
	// white pwm
	else if (strstr(buffer,"we") != NULL)
		msg_data.msg_text[0]=MSG_LEDW_ENABLE;
	else if (strstr(buffer,"wd") != NULL)
		msg_data.msg_text[0]=MSG_LEDW_DISABLE;
	else if (strstr(buffer,"wb") != NULL)
		msg_data.msg_text[0]=MSG_LEDW_BREATHING;
	// breathing freq
	else if (strstr(buffer,"b3") != NULL)
		msg_data.msg_text[0]=MSG_CYCLE_TIME_3;
	else if (strstr(buffer,"b4") != NULL)
		msg_data.msg_text[0]=MSG_CYCLE_TIME_4;
	else if (strstr(buffer,"b5") != NULL)
		msg_data.msg_text[0]=MSG_CYCLE_TIME_5;
	else if (strstr(buffer,"b6") != NULL)
		msg_data.msg_text[0]=MSG_CYCLE_TIME_6;
	else if (strstr(buffer,"b7") != NULL)
		msg_data.msg_text[0]=MSG_CYCLE_TIME_7;
	else 
		printf("Unknow args %s\n", buffer);

    // 设置消息数据
    msg_data.msg_type = 1;
    //strncpy(msg_data.msg_text, buffer, MSG_SIZE);

    // 发送消息
    if (msgsnd(msgid, &msg_data, sizeof(msg_data.msg_text), 0) == -1) {
        perror("msgsnd");
        exit(EXIT_FAILURE);
    }

    printf("Data sent: %s", msg_data.msg_text);

    return EXIT_SUCCESS;
}

