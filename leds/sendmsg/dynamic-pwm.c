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
#include <sys/stat.h>
#include <mqueue.h>
#include "pwm_message.h"

#define PWM_PREFIX	"/sys/class/pwm/pwmchip"
#define PWM_FREQ	5000 // 5 kHz
#define PWM_POLARITY	1 // normal

unsigned int period[2]={0};

/*
#define PWM0_INITIAL	"/sys/class/pwm/pwmchip0/export"
#define PWM0_RELEASE	"/sys/class/pwm/pwmchip0/unexport"
#define PWM0_ENABLE	"/sys/class/pwm/pwmchip0/pwm0/enable"
#define PWM0_PERIOD	"/sys/class/pwm/pwmchip0/pwm0/period"
#define PWM0_POLARITY	"/sys/class/pwm/pwmchip0/pwm0/polarity"
#define PWM0_DUTY	"/sys/class/pwm/pwmchip0/pwm0/duty_cycle"

#define PWM1_INITIAL	"/sys/class/pwm/pwmchip1/export"
#define PWM1_RELEASE	"/sys/class/pwm/pwmchip1/unexport"
#define PWM1_ENABLE	"/sys/class/pwm/pwmchip1/pwm0/enable"
#define PWM1_PERIOD	"/sys/class/pwm/pwmchip1/pwm0/period"
#define PWM1_POLARITY	"/sys/class/pwm/pwmchip1/pwm0/polarity"
#define PWM1_DUTY	"/sys/class/pwm/pwmchip1/pwm0/duty_cycle"
*/

static int pwm_initial(int id, int pol) {
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
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	if (pol)
		fprintf(fpwm, "%s", "normal");
	else
		fprintf(fpwm, "%s", "inversed");
	fclose(fpwm);
	printf("PWM%d polarity is %s\n",id, pol ? "normal":"inversed");

	return 0;
}
static int pwm_enable(int id, int en) {
	char path[64]={0};
	FILE *fpwm;
	sprintf(path,"%s%d/pwm0/enable",PWM_PREFIX,id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	if (en)
		fprintf(fpwm, "%d", 1);
	else
		fprintf(fpwm, "%d", 0);
	fclose(fpwm);
	printf("PWM%d %s done\n",id, en ? "enable":"disable");
	return 0;
}
static int pwm_set_period(int id, unsigned int freq) {
	char path[64]={0};
	FILE *fpwm;
	if ( (freq > 1000000) || (id>=2) || (freq < 100))
		printf("ERROR : freq over spec\n");

	period[id] = 1000000000/freq;

	sprintf(path,"%s%d/pwm0/period",PWM_PREFIX,id);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%d", period[id]);
	fclose(fpwm);
	printf("PWM%d period=%d freq=%d\n",id, period[id],freq);
	return 0;
}
static int pwm_set_duty(int id, unsigned int percent) {
	char path[64]={0};
	unsigned val=0;
	FILE *fpwm;
	if ( (percent > 100) || (id>=2) || (period[id]==0))
		printf("ERROR : duty params errot, percent=%d, id=%d, period=%d\n", percent, id, period[id]);
	val = (period[id]*percent/100);
	sprintf(path,"%s%d/pwm0/duty_cycle",PWM_PREFIX,id,val);
	if (NULL == (fpwm=fopen(path,"w")) ) {
		printf("Can't open export file: %s\n", path);
		return 1;
	}
	fprintf(fpwm, "%d", val);
	fclose(fpwm);
	printf("PWM%d period=%d duty=%d \n",id, period[id],val);
	return 0;	
}

void *receiveMessage(void *arg) {
	mqd_t mq;
	char buffer[MAX_MSG_SIZE];
	unsigned msg=MSG_NONE;


	// Open the message queue for receiving
	mq = mq_open(QUEUE_NAME, O_RDONLY);
	if (mq == (mqd_t)-1) {
		perror("mq_open in receiver");
		exit(EXIT_FAILURE);
	}

#if 1
	while (msg != MSG_EXIT) {
		if (mq_receive(mq, buffer, MAX_MSG_SIZE, NULL) == -1) {
			perror("mq_receive");
			break;
		}

		printf("Received message: %s\n", buffer);

		if (strcmp(buffer, "exit") == 0) {
			break;
		}		
	}

#else
    while (1) {
        if (mq_receive(mq, buffer, MAX_MSG_SIZE, NULL) == -1) {
            perror("mq_receive");
            break;
        }

        printf("Received message: %s\n", buffer);

        if (strcmp(buffer, "exit") == 0) {
            break;
        }
    }
#endif
    // Close the message queue
	mq_close(mq);

	return NULL;
}

int main(int argc, char **argv)
{
	unsigned int duty, freq=PWM_FREQ;
	pthread_t receiverThread;
	pwm_initial(0,PWM_POLARITY);
	pwm_set_period(0,freq);
	pwm_set_duty(0,0);
	pwm_enable(0,1);

	pwm_initial(1,PWM_POLARITY);
	pwm_set_period(1,freq);
	pwm_set_duty(1,0);
	pwm_enable(1,1);



    // Create receiver thread
    if (pthread_create(&receiverThread, NULL, receiveMessage, NULL) != 0) {
        fprintf(stderr, "Error creating receiver thread.\n");
        return EXIT_FAILURE;
    }

    // Wait for receiver thread to finish
    pthread_join(receiverThread, NULL);

    return EXIT_SUCCESS;
	return 0;
}

