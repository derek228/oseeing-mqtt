#ifndef MESSAGE_QUEUE_H
#define MESSAGE_QUEUE_H

#define PWM_QUEUE_NAME "/pwm_msg_queue"
#define MAX_MSG_SIZE 256
enum {
MSG_NONE = 0, 
MSG_ENABLE,
MSG_DISABLE,
MSG_CYCLE_TIME, // sec
MSG_EXIT
};
#endif /* MESSAGE_QUEUE_H */

