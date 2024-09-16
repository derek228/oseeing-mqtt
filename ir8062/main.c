#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include "spidev.h"
#include "ini-parse.h"
#include "../leds/pwm_message.h"
#include "led_control.h"
#include "ethernet.h"
#include "mi48.h"
#include "rs485.h"
#include "mqtt.h"
#define FW_VERSION	"1.00.04"
static void parse_opts(int argc, char *argv[])
{
	//char *macno;
	//int i=0;
	while (1) {
		int c;
		c = getopt(argc, argv, "m:");
		if (c == -1) {
			break;
		}
		switch (c) {
			case 'm':
				eth_set_fake_mac(optarg);
				printf("Set fack MAC address = %s, len=%d\n",optarg,strlen(optarg));
				break;
			default:
				printf("Unknow args, use original MAC NO=%s\n",optarg);
				break;
		}
	}
}
static int factory_test() {
	// To Do : Check factory gpio, if not factory test mode, return 0
	return 0; 
	while (1) {
		printf("Start factory mode\n");
		sleep(1);
	}
}
static void monitor_temperature() {
	// INI file configuartion parse.
	while (rs485_init()<0) {
		printf("ERROR : Can't open RS485 port\n");
		sleep(1);
	}
	// Check dhcp if support ethernet
	eth_mac_config();
	#if 0
	while (ethernet_init() < 0) {
		printf("ERROR : DHCP IP address not found\n");
		sleep(1);
	}
	#endif
	mqtt_init();
	// start system LEDs control
	led_msg_init();
	printf("Post MAC address = %s\n", eth_get_mac());
	// MI48 thermal sensor initial
	mi48_init();
	while (1) {
		if (mi48_scan()) {
			led_heartbit();
		}
	}
}
int main(int argc, char *argv[]) {
	//pthread_t tid_sensor;
	// Show Oseeing firmware version
	printf("FW Version : %s\n", FW_VERSION);
	parse_opts(argc, argv);
	system("ifconfig eth0 up");
	if (factory_test()) {
		return 0;
	}
	else {
		monitor_temperature();
	}
	return 0;
}

