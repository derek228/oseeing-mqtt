#ifndef __LEDS__
#define __LEDS__

enum {
LED_R = 0,
LED_G,
LED_B,
LED_W,
LED_NUMS
};
enum {
LED_BREATHING_OFF = 0,
LED_BREATHING_UP,
LED_BREATHING_DOWN
};
#define PWM_LED_TYPE 0 // x80000000
#define GPIO_LED_TYPE 0x80000000
#define GPIO_LED_R	193
#define GPIO_LED_G	79
#define GPIO_LED_B	75
#define PWM_LED_W	0
void led_set(int id, int en, int breathing, int blanking);
void led_blanking_set(int id, int delay); // delay in 1~3 sec
#endif // __LEDS__
