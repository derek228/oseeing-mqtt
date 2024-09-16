#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h> 
#include <fcntl.h> 
#include <termios.h>  
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <pthread.h>
#include <dirent.h>
#include <unistd.h>
#include <ctype.h>
#include "MQTTClient.h"
#include "ethernet.h"

//#define ADDRESS     "tcp://test.mosquitto.org:1883"
#define ADDRESS     "tcp://192.168.100.70:1883"
#define CLIENTID    "ExampleClientPub"
#define TOPIC       "oseeing/thermal"
#define PAYLOAD     "Hello MQTT"
#define QOS         1
#define TIMEOUT     10000L
#define MQTT_DISCONNECTED   0
#define MQTT_CONNECTED      1
#define RJ45_CONN_STATUS    "/sys/class/net/eth0/carrier"
typedef struct {
    char conn;
    char addr[128];
    int addr_len;
    char qos;
    int timeout;
}mqtt_info_t;

pthread_t mqtt_tid;
mqtt_info_t mqtt_info;
MQTTClient client;
MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;

static int oseeing_pidkill(char *program_name) {
	//char *program_name = "ir8062";
	int pid = -1;
	DIR *dir;
	struct dirent *ent;
	char buf[512];

	if ((dir = opendir("/proc")) != NULL) {
		while ((ent = readdir(dir)) != NULL) {
			if (isdigit(*ent->d_name)) {
				sprintf(buf, "/proc/%s/cmdline", ent->d_name);
				FILE *fp = fopen(buf, "r");
				if (fp != NULL) {
					fgets(buf, sizeof(buf), fp);
					fclose(fp);
					if (strstr(buf, program_name) != NULL) {
						pid = atoi(ent->d_name); 
						break;
					}
				}
			}
        	}
        	closedir(dir);
	}

	if (pid != -1) {
		printf("PID of process '%s': %d\n", program_name, pid);
	} else {
		printf("Process '%s' not found\n", program_name);
		return -1;
	}
	int result = kill(pid, SIGKILL);
	if (result == 0) {
		printf("Process with PID %d killed successfully.\n", pid);
	} else {
		perror("Error killing process");
		return -1;
	}

	return 0;
}

int is_rj45_connected() {
    FILE *file=NULL;
    char status;
    static unsigned int check_cnt=0;
    static char is_connected =0;
    check_cnt++;
    if( (check_cnt % 3) != 0) {
        if (is_connected)
            return 1;
        else
            return -1;
    }
    printf("Check RJ45 connection status %d\n",check_cnt);
    file = fopen(RJ45_CONN_STATUS,"rb");
    if (file == NULL) {
        printf("Can't open %s\n",RJ45_CONN_STATUS);
        return -1;
    }
	size_t read = fread(&status, 1, 1, file);
	if (read != 1) {
		fclose(file);
		printf("Read(%d) RJ45 data(%d) fail\n", read, status);
        system("ifconfig eth0 up");
		return -1;
	}
    fclose(file);
    if (status == '1') {
        printf("RJ45 cable is connected...\n");
        ethernet_init();
        is_connected = 1;
/*        if (mqtt_info.conn == MQTT_DISCONNECTED) {
            printf("Re-connect MQTT server \n");
            return mqtt_connect();
        }
        else*/
        return 1;
    }
    else {
        printf("RJ45 cable is disconnected...\n");
        oseeing_pidkill("udhcpc");
        system("ifconfig eth0 down");
        is_connected = 0;
/*        if (mqtt_info.conn == MQTT_CONNECTED) {
            mqtt_disconnect();
            return -1;
        }*/
        return -1;
    }
}

int mqtt_disconnect() {
    printf("MQTT disconnected\n");
    MQTTClient_disconnect(client, 10000);
    //MQTTClient_destroy(&client);  
    mqtt_info.conn=MQTT_DISCONNECTED;
    return 0;  
}

int mqtt_connect() {
    // 連接到 Broker
    // 設定連接選項
    int rc;
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect, return code %d\n", rc);
        //exit(EXIT_FAILURE);
        return -1;
    }
    mqtt_info.conn = MQTT_CONNECTED;
    return 1;
}
void *mqtt_thread(void *arg) {
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    MQTTClient_deliveryToken token;
    int rc;
    while (1) {
        if (is_rj45_connected() < 0) {
            if (mqtt_info.conn == MQTT_CONNECTED)
                mqtt_disconnect();
            sleep(1);
            continue;
        }
        else {
            if (mqtt_info.conn==MQTT_DISCONNECTED ) {
                if (mqtt_connect() < 0) {
                    sleep(1);
                    continue;
                }
            }
            if (MQTTClient_isConnected(client)==0) {
                printf("MQTT server disconnect, try to re-connect...\n");
                mqtt_info.conn = MQTT_DISCONNECTED;
                continue;
            }
        }
        // 設定消息

        pubmsg.payload = (void*)PAYLOAD;
        pubmsg.payloadlen = (int)strlen(PAYLOAD);
        pubmsg.qos = QOS;
        pubmsg.retained = 0;

    // 發布消息
        MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
        printf("Waiting for up to %d seconds for publication of %s\n"
           "on topic %s for client with ClientID: %s\n",
           (int)(TIMEOUT / 1000), PAYLOAD, TOPIC, eth_get_mac());
        rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
        printf("Message with delivery token %d delivered\n", token);
        sleep(1);
    }
    //return;
}
void mqtt_thread_destory() {
    pthread_cancel(mqtt_tid);
    pthread_join(mqtt_tid, NULL);  
}

int mqtt_thread_create() {
    if (pthread_create(&mqtt_tid, NULL, mqtt_thread, NULL) != 0) {
        perror("mqtt pthread_create");
        return -1;
    }
    return 0;
}

int mqtt_init()
{
    //MQTTClient client;
    //MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    //MQTTClient_message pubmsg = MQTTClient_message_initializer;
    //MQTTClient_deliveryToken token;
    is_rj45_connected();
    // 創建 MQTT 客戶端
    MQTTClient_create(&client, ADDRESS, eth_get_mac(), MQTTCLIENT_PERSISTENCE_NONE, NULL);
    mqtt_info.conn = MQTT_DISCONNECTED;
    mqtt_thread_create();
/*
    // 設定消息
    pubmsg.payload = (void*)PAYLOAD;
    pubmsg.payloadlen = (int)strlen(PAYLOAD);
    pubmsg.qos = QOS;
    pubmsg.retained = 0;

    // 發布消息
    MQTTClient_publishMessage(client, TOPIC, &pubmsg, &token);
    printf("Waiting for up to %d seconds for publication of %s\n"
           "on topic %s for client with ClientID: %s\n",
           (int)(TIMEOUT / 1000), PAYLOAD, TOPIC, CLIENTID);
    rc = MQTTClient_waitForCompletion(client, token, TIMEOUT);
    printf("Message with delivery token %d delivered\n", token);

    斷開連接並清理
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    */
    return 0;// rc;
}