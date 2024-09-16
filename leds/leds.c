/* GPIO Driver Test/Example Program
 *
 * Compile with:
 *  gcc -s -Wall -Wstrict-prototypes gpio.c -o gpiotest
 *
 *
 * Note :
 *   PORT NAME[PIN] = GPIO [id]	
 *   PORTA[ 0]      = gpio[ 0x00]
 *   PORTA[ 1]      = gpio[ 0x01]	  
 *                  :
 *   PORTA[31]      = gpio[ 0x1F]
 *   PORTB[ 0]      = gpio[ 0x20]
 *                  :
 *   PORTB[31]      = gpio[ 0x3F]
 *                  :
 *                  :
 *                  :
 *   PORTI[ 0]      = gpio[ 0xC0]
 *                  :
 *                  :
 *   PORTI[31]      = gpio[ 0xDF]
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <errno.h>
#include "pwm_message.h"
#include "leds.h"

#define PWM_PREFIX	"/sys/class/pwm/pwmchip"
#define GPIO_PREFIX	"/sys/class/gpio/"

#define DEFALUT_BREATHING_CYCLE	3 // 5sec
// active low
#define DUTY_BREATHING_MAX	99// 80
#define DUTY_BREATHING_MIN	50 // 30
// active high
//#define DUTY_BREATHING_MAX	50//99// 80
//#define DUTY_BREATHING_MIN	0//50 // 30
#define PWM_FREQ	5000 // 5 kHz

#define PWM_POLARITY	1 // normal
#define DUTY_FULL	100
#define BREATHING_DELAY(s)	(s*1000000/(DUTY_BREATHING_MAX-DUTY_BREATHING_MIN))
#define PWM_PERIOD	(1000000000/PWM_FREQ) // .period
#define PWM_FLASH_DELAY	10000 // us .breaty_delay
#define PWM_FPS	(1000000/PWM_FLASH_DELAY)
#define LED_BLANKING_COUNT(s)	(s*PWM_FPS) // gpio mode .breath_delay
#define PWM_ACTIVE_PERIOD	( (DUTY_BREATHING_MAX-DUTY_BREATHING_MIN)*PWM_PERIOD/100)  // p=period, f=pwmfps, s=sec
#define PWM_ACTIVE_PERIOD_MIN	(DUTY_BREATHING_MIN*PWM_PERIOD/100) // max duty
#define PWM_ACTIVE_PERIOD_MAX	(DUTY_BREATHING_MAX*PWM_PERIOD/100) // min duty 
#define PWM_DUTY_TOTAL_STEP	(DEFALUT_BREATHING_CYCLE*PWM_FPS)
#define PWM_DUTY_STEP		(PWM_ACTIVE_PERIOD/PWM_DUTY_TOTAL_STEP) // duty + step
#define PWM_ACTIVE_PERIOD_STEP	(PWM_ACTIVE_PERIOD/PWM_DUTY_TOTAL_STEP)


/* led gpio config 
   bit 31 : pwm type
   bit 30 : gpio type
   bit [29..28] : led blanking freq : 0~3 sec
   bit [27..8] : pwm freq 0~0xFFFFF (1,048,575)Hz
   bit [7..0] : gpio/pwm num
*/
#define GET_PWM_FREQ(f)		((f&0x0fffff00) >> 8)
#define GET_BLANKING_FREQ(f)	((f&0x30000000) >> 28)
#define GET_GPIO_INVERSE(inv)	((inv&0x40000000) >> 30)
#define LED_BLANKING_1S	0x10000000 
#define LED_BLANKING_2S	0x20000000
#define LED_BLANKING_3S	0x30000000
#define DEFAULT_LED_PWM_FREQ (PWM_FREQ<<8)
#define INVERSE_ENABLE (1 << 30)
#define DISABLE	0
#define ENABLE	1
#define LED_R_FREQ	2 //sec
#define LED_G_FREQ	1 //sec
#define LED_B_FREQ	1 //sec
#define LED_Y_FREQ	1 //sec

#define logd(  dbg, x, y... ) do{ \
    if(dbg == 1 )\
    printf( x, ##y ); }while(0)

static int heartbit_alarm = 0;
static int heartbit_cnt=0;
static int leds_debug=0;

static void leds_log_print() {
    const char *filename = "/mnt/mtdblock1/ledlog";

    // 检查文件是否存在
    if (access(filename, F_OK) != -1) {
        leds_debug=1;
    } else {
        leds_debug=0;
    }
}

static unsigned int led_config_table[LED_NUMS] = {GPIO_LED_TYPE | INVERSE_ENABLE | LED_BLANKING_1S |GPIO_LED_R, 
						GPIO_LED_TYPE | INVERSE_ENABLE | LED_BLANKING_2S | GPIO_LED_G, 
						GPIO_LED_TYPE | LED_BLANKING_1S | GPIO_LED_B, 
						PWM_LED_TYPE | DEFAULT_LED_PWM_FREQ | PWM_LED_W};
typedef struct {
	unsigned int type;
	int id;
	unsigned int duty; // in gpio mode , it is blanking count
	unsigned int period;
	//unsigned int direction; // 0 : up  , 1: down
	int pwm_en;
	int breath_en;	// in gpio mode, it is blanking toggle signal
	int led_en;
	int led_blanking;
	int led_freq;
	int breath_delay; // in gpio mode, it is blanking max count
	int inverse;
}led_state_t;

typedef struct {
	unsigned int duty_step;
	unsigned int duty_step_max;
	unsigned int duty_step_min;
	unsigned int period_step;
	unsigned int period_max;
	unsigned int period_min;
}breathing_param_t;
breathing_param_t breathing_t;
led_state_t led_t[LED_NUMS]; // 0:pwm0 , 1:pwm1, 2&3: gpio
unsigned int period[2]={0};
unsigned int breath_delay=PWM_FLASH_DELAY; // BREATHING_DELAY(DEFALUT_BREATHING_CYCLE); // s

static void print_leds() {
	int i;
	for (i=0; i<LED_NUMS; i++)
	{
		printf("LED%d: type=0x%x, en=%d, breath=%d,duty=%d\n", i, led_t[i].type, led_t[i].pwm_en, led_t[i].breath_en,led_t[i].duty, led_t[i]);
	}
}
static int pwm_set_period(int id) {
	char path[64]={0};
	FILE *fpwm;
	if ((led_t[id].type & GPIO_LED_TYPE)) {
		printf("LED%d not pwm mode\n",id);
		return 1;
	}
	if ( (led_t[id].led_freq > 1000000) || (led_t[id].led_freq < 100)) {
		printf("ERROR : freq over spec\n");
		return 1;
	}
	led_t[id].period=1000000000/led_t[id].led_freq;
	sprintf(path,"%s%d/pwm0/period",PWM_PREFIX,led_t[id].id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%d", led_t[id].period);
	fclose(fpwm);

	//printf("PWM%d period=%d freq=%d\n",led_t[id].id, led_t[id].period,led_t[id].led_freq);
	return 0;
}
static int gpio_led_init(int id, int val) {
	char path[64]={0};
	FILE *fgpio;
	// export gpio num
	sprintf(path,"%s/export",GPIO_PREFIX);
	if (NULL == (fgpio=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fgpio,"%d",id);
	fclose(fgpio);
	// set gpio in/out
	sprintf(path,"%s/gpio%d/direction",GPIO_PREFIX,id);
	if (NULL == (fgpio=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fgpio,"%s","out");
	fclose(fgpio);
	// set gpio value
	sprintf(path,"%s/gpio%d/value",GPIO_PREFIX,id);
	if (NULL == (fgpio=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fgpio,"%d",val);
	fclose(fgpio);
	return 0;
}

static int pwm_led_init(int id) {
	char path[64]={0};
	FILE *fpwm;
	sprintf(path,"%s%d/export",PWM_PREFIX,id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%d", 0);
	fclose(fpwm);
	printf("PWM%d initial done\n",id);
	sprintf(path,"%s%d/pwm0/polarity",PWM_PREFIX,id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open PWM export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%s", "normal"); // normal, inversed
	fclose(fpwm);
	return 0;
}
static int set_breathing_params() {
	breathing_t.duty_step = PWM_DUTY_TOTAL_STEP;
	breathing_t.duty_step_max = PWM_ACTIVE_PERIOD_MAX;
	breathing_t.duty_step_min = PWM_ACTIVE_PERIOD_MIN;
	breathing_t.period_step = PWM_DUTY_STEP;
	breathing_t.period_max = PWM_ACTIVE_PERIOD_MAX;
	breathing_t.period_min = PWM_ACTIVE_PERIOD_MIN;
	printf("Breathing duty: step=%d ,max=%d, min=%d\n",breathing_t.duty_step,breathing_t.duty_step_max, breathing_t.duty_step_min);
	printf("Breathing period: step=%d ,max=%d, min=%d\n",breathing_t.period_step,breathing_t.period_max, breathing_t.period_min);	
}
static int leds_init() {
	int i=0;
	for (i=0; i< LED_NUMS; i++) {
		if (led_config_table[i]&GPIO_LED_TYPE) { // gpio led
			led_t[i].type=led_config_table[i] & GPIO_LED_TYPE;
			led_t[i].id = led_config_table[i] & 0xff;
			led_t[i].led_freq=GET_BLANKING_FREQ(led_config_table[i]);
			led_t[i].inverse = GET_GPIO_INVERSE(led_config_table[i]);
			led_t[i].breath_delay=LED_BLANKING_COUNT(led_t[i].led_freq); // (led_t[i].led_freq*1000000)/BREATHING_DELAY(DEFALUT_BREATHING_CYCLE);
			led_t[i].led_en = DISABLE;
			led_t[i].duty=0; // toggle count
			led_t[i].breath_en=0; // use for gpio togle signal			
			if (led_t[i].led_freq)	led_t[i].led_blanking = ENABLE;
			else led_t[i].led_blanking = DISABLE;
			if (led_t[i].inverse)
				gpio_led_init(led_t[i].id, 1);
			else 
				gpio_led_init(led_t[i].id, 0);
			logd(leds_debug,"GPIO LED%d init: blanking=%d, freq=%d, gpio=%d, inverse=%d, toggle_cnt=%d\n",i,led_t[i].led_blanking, led_t[i].led_freq, led_t[i].id, led_t[i].inverse, led_t[i].breath_delay);
		}
		else  {
			led_t[i].type=led_config_table[i] & PWM_LED_TYPE;
			led_t[i].id = led_config_table[i] & 0xff;
			led_t[i].led_freq = GET_PWM_FREQ(led_config_table[i]);
			led_t[i].duty=0; // DUTY_BREATHING_MIN;
			led_t[i].breath_en = DISABLE;
			led_t[i].pwm_en = DISABLE;
			led_t[i].breath_delay = PWM_FLASH_DELAY; // BREATHING_DELAY(DEFALUT_BREATHING_CYCLE); // s
			if (pwm_led_init(led_t[i].id) == 0) {
				pwm_set_period(i);
			}
			else 
				printf("ERROR PWM LED %d init failed\n", i);
			logd(leds_debug,"PWM LED%d init: blanking=%d, freq=%d, gpio=%d\n",i,led_t[i].led_blanking, led_t[i].led_freq, led_t[i].id);
			// set breathing params
			set_breathing_params();
		}
	}
	return 0;
}



static int pwm_enable(int id, int en) {
	char path[64]={0};
	FILE *fpwm;
	led_t[LED_W].pwm_en=en;
	sprintf(path,"%s%d/pwm0/enable",PWM_PREFIX,led_t[id].id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	if (en)
		fprintf(fpwm, "%d", 1);
	else
		fprintf(fpwm, "%d", 0);
	fclose(fpwm);
	logd(leds_debug,"PWM%d %s done\n",id, en ? "enable":"disable");
	return 0;
}

static int pwm_set_duty(int id, unsigned int percent) {
	char path[64]={0};
	unsigned int val=0;
	FILE *fpwm;
	
	if ( (led_t[id].type&GPIO_LED_TYPE) ) {
		printf("LED%d not pwm mode\n",id);
		return 1;
	}
	if (percent > breathing_t.duty_step) 
		percent = breathing_t.duty_step;
	//printf("PWM%d duty=%d, period=%d\n", led_t[id].id, percent,led_t[id].period);
	// val = (led_t[id].period*percent/100);
	val = breathing_t.period_min+(percent*breathing_t.period_step);
	sprintf(path,"%s%d/pwm0/duty_cycle",PWM_PREFIX,led_t[id].id,val);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%d", val);
	fclose(fpwm);
	//printf("PWM%d period=%d duty=%d \n",id, led_t[id].period,val);
	return 0;	
}
static int set_led_gpios(int id, int value) {
	char path[64]={0};
	FILE *fgpio;
	// set gpio value
	sprintf(path,"%s/gpio%d/value",GPIO_PREFIX,id);
	if (NULL == (fgpio=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fgpio,"%d",value);
	fclose(fgpio);
	return 0;
}
static void led_blanking() {
	int i=0;
//	int val=0;
	for (i=0; i<LED_NUMS; i++) {
		if (led_t[i].type != GPIO_LED_TYPE) continue;
		else if (led_t[i].led_en == 0) continue;
		else if (led_t[i].led_blanking) {
			led_t[i].duty++;
			if (led_t[i].duty>=led_t[i].breath_delay) {
				logd(leds_debug,"LED%d toggle: count=%d, max=%d, delay=%d\n",i, led_t[i].duty, led_t[i].breath_delay, led_t[3].breath_delay);
				led_t[i].duty = 0;
				led_t[i].breath_en = !led_t[i].breath_en;
				set_led_gpios(led_t[i].id, led_t[i].breath_en);
			}
		}
	}
}
static void breathing_lignt(int id) {

	if ( (led_t[id].breath_en==LED_BREATHING_OFF) && (led_t[id].pwm_en) ) { // No breathing, led on
		pwm_set_duty(id, 100);
	}
	else if (led_t[id].pwm_en == 0) {
		pwm_set_duty(id,0);
	}
	else {
		//printf("Breathing id %d: duty=%d, breath=%d\n", id, led_t[id].duty, led_t[id].breath_en);
		pwm_set_duty(id, led_t[id].duty);
		if (led_t[id].breath_en == LED_BREATHING_UP) {
			led_t[id].duty++;
			//printf("Breathing id %d: duty=%d, breath=%d\n", id, led_t[id].duty, led_t[id].breath_en);
//			if (led_t[id].duty>DUTY_BREATHING_MAX) {
			if (led_t[id].duty>breathing_t.duty_step) {
				led_t[id].duty=breathing_t.duty_step;
				led_t[id].breath_en = LED_BREATHING_DOWN;
			}
		}
		else if (led_t[id].breath_en == LED_BREATHING_DOWN) {
			led_t[id].duty--;
			//printf("Breathing id %d: duty=%d, breath=%d\n", id, led_t[id].duty, led_t[id].breath_en);
			if (led_t[id].duty<=0) {
				led_t[id].duty=1;
				led_t[id].breath_en = LED_BREATHING_UP;
			}
		}
		else 
			printf("ERROR : can't set breathing_light, breath=%d, duty=%d\n",led_t[id].breath_en, led_t[id].duty);
	}
}
static void pwm_set(int id, int en, int breath)
{
	printf("Set PWM%d en=%d breath=%d\n", id, en, breath);
	led_t[id].duty = 0; // DUTY_BREATHING_MIN;
	led_t[id].pwm_en = en;
	led_t[id].breath_en=breath;
}
static int heartbit_monitor() {
	if (heartbit_cnt > heartbit_alarm) {
		
		//printf("ERROR : Thermal sensor is stoped\n");
		led_set(LED_W,0,0,0);
		return 1;
	}	
	heartbit_cnt++;
	return 0;
}
static void heartbit_init(int delay) {
	heartbit_alarm=(delay*1000000)/PWM_FLASH_DELAY; // BREATHING_DELAY(DEFALUT_BREATHING_CYCLE);;
	heartbit_cnt = 0;
	printf("heartbit_alarm = %d\n",heartbit_alarm);
}

void show_leds(int msg) {
	switch (msg) {
	case MSG_LEDR_ENABLE:
		led_set(LED_R,1,0,0); // color, enable, breathing, blanking
		led_set(LED_G,0,0,0);
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDR_DISABLE:
		led_set(LED_R,0,0,0);
		break;
	case MSG_LEDR_BLANKING:
		led_set(LED_R,1,0,LED_R_FREQ);
		led_set(LED_G,0,0,0);
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDG_ENABLE:
		led_set(LED_R,0,0,0);
		led_set(LED_G,1,0,0);
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDG_DISABLE:
		led_set(LED_G,0,0,0);
		break;
	case MSG_LEDG_BLANKING:
		led_set(LED_R,0,0,0);
		led_set(LED_G,1,0,LED_G_FREQ);
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDB_ENABLE:
		led_set(LED_R,0,0,0);
		led_set(LED_G,0,0,0);
		led_set(LED_B,1,0,0);
		break;
	case MSG_LEDB_DISABLE:
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDB_BLANKING:
		led_set(LED_R,0,0,0);
		led_set(LED_G,0,0,0);
		led_set(LED_B,1,0,LED_B_FREQ);
		break;
	case MSG_LEDW_ENABLE:
		led_set(LED_W,1,0,0);
		break;
	case MSG_LEDW_DISABLE:
		led_set(LED_W,0,0,0);
		break;
	case MSG_LEDW_BREATHING:
		led_set(LED_W,1,1,0);
		break;
	case MSG_LEDY_ENABLE:
		led_set(LED_R,1,0,0);
		led_set(LED_G,1,0,0);
		led_set(LED_B,0,0,0);
		break;
	case MSG_LEDY_DISABLE:
		led_set(LED_R,0,0,0);
		led_set(LED_G,0,0,0);
		break;
	case MSG_LEDY_BLANKING:
		led_set(LED_R,1,0,LED_Y_FREQ);
		led_set(LED_G,1,0,LED_Y_FREQ);
		led_set(LED_B,0,0,0);
		break;
/*	case MSG_CYCLE_TIME_4:
		led_t[LED_W].breath_delay = BREATHING_DELAY(4);
		break;
	case MSG_CYCLE_TIME_5:
		led_t[LED_W].breath_delay = BREATHING_DELAY(5);
		break;
	case MSG_CYCLE_TIME_6:
		led_t[LED_W].breath_delay = BREATHING_DELAY(6);
		break;
	case MSG_CYCLE_TIME_7:
		led_t[LED_W].breath_delay = BREATHING_DELAY(7);
		break;
	case MSG_CYCLE_TIME_3:
		led_t[LED_W].breath_delay = BREATHING_DELAY(3);
		break;
*/
	case MSG_HEART_BIT:
		heartbit_cnt=0;
		break;
	case MSG_EXIT:
		exit(1);
		break;
	default :
		break;
	}
}
void *receiveMessage(void *arg) {

	int msgid;
	struct message msg_data;
	unsigned int msg=MSG_NONE;
	// 创建或获取消息队列
	msgid = msgget(MSG_KEY, 0666 | IPC_CREAT);
	if (msgid == -1) {
		perror("msgget");
		exit(EXIT_FAILURE);
	}

	// 获取相同的消息队列
	msgid = msgget(MSG_KEY, 0666);
	if (msgid == -1) {
		perror("msgget");
		exit(EXIT_FAILURE);
	}

	// 接收消息
	while (msg != MSG_EXIT) {
		if (msgrcv(msgid, &msg_data, sizeof(msg_data.msg_text), 1, IPC_NOWAIT) == -1) {
			if (errno == ENOMSG) {
				leds_log_print();
				//printf("No message in the queue\n");
				breathing_lignt(LED_W);
				led_blanking();
				// heartbit_monitor(); // 
				usleep(led_t[LED_W].breath_delay);
				//sleep(1);
			}
			else {
				perror("msgrcv");
				exit(EXIT_FAILURE);
			}
		}
		else {
			//print_leds();
			show_leds(msg_data.msg_text[0]);
			//printf("Data received: 0x%d, delay=%d\n", msg_data.msg_text, led_t[LED_W].breath_delay);
		}
	}
    return EXIT_SUCCESS;
}
/*	id:color id , LED_R...
	en: enable 
	breathing: PWM breathing enable
	blanking : blanking frequency 
*/
void led_set(int id, int en, int breathing, int blanking) {
	logd(leds_debug,"Set LED%d, en=%d, breathing=%d, blanking=%d\n", id, en, breathing, blanking);
	if (led_t[id].type & GPIO_LED_TYPE) { // gpio led
		led_t[id].led_en = en;
		if (blanking) {
			led_t[id].led_blanking = 1;
			if (led_t[id].inverse)
				led_t[id].breath_en=1;
			else
				led_t[id].breath_en=0; // led flash toggle 
			led_t[id].duty=0; // led flash cnt
			led_t[id].led_freq=blanking;;
			led_t[id].breath_delay=LED_BLANKING_COUNT(led_t[id].led_freq); 
			// (led_t[id].led_freq*1000000)/BREATHING_DELAY(DEFALUT_BREATHING_CYCLE); // led flash max count
		}
		else 
			led_t[id].led_blanking=0;
		if (led_t[id].inverse)
			en = !en;
		set_led_gpios(led_t[id].id, en);
	}
	else { // pwm led
		led_t[id].pwm_en = en;
		led_t[id].breath_en = breathing;
		led_t[id].duty=DUTY_BREATHING_MIN;
		pwm_enable(id,en);
	}
}
void led_blanking_set(int id, int delay) { // delay in 1~3 sec
	if ((led_t[id].type && GPIO_LED_TYPE) ==0)
	{
		printf("ERROR : LED%d not GPIO mode\n",id);
	}
	else 
		printf("Set LED%d, blanking in %d sec\n",id, delay);
	led_t[id].breath_en=0;
	led_t[id].duty=0;
	led_t[id].led_blanking=1;
	led_t[id].led_freq=delay;
	led_t[id].breath_delay=LED_BLANKING_COUNT(led_t[id].led_freq); // (led_t[id].led_freq*1000000)/BREATHING_DELAY(DEFALUT_BREATHING_CYCLE);
	led_t[id].led_en=1;
}
int main(int argc, char **argv)
{
	pthread_t receiverThread;
	if (leds_init()) {
		printf("ERROR : LED init failure...\n");
		return EXIT_FAILURE;
	}
	led_set(LED_B, 1, 1, 1);
	heartbit_init(30);
	// Create receiver thread
	if (pthread_create(&receiverThread, NULL, receiveMessage, NULL) != 0) {
		fprintf(stderr, "Error creating receiver thread.\n");
		return EXIT_FAILURE;
	}

	// Wait for receiver thread to finish
	pthread_join(receiverThread, NULL);

	return 0;
}



