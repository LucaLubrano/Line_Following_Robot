#include "pin_config.h"
#include "macros.h"

/*_______________________________________________________________*/

/* Function Prototypes */

void timer_init(void);

void IR_init(void);

void PWM_init(void);

void LED_init(void);

void motor_init(void);

/*_______________________________________________________________*/

/* Function Declarations */

void IR_init(void){
  // Set all to inputs
    CLR_BIT(DDRF, IR_SENSOR_1);
    CLR_BIT(DDRF, IR_SENSOR_2);
    CLR_BIT(DDRF, IR_SENSOR_3);
    CLR_BIT(DDRF, IR_SENSOR_4);
    CLR_BIT(DDRB, IR_SENSOR_5);
    CLR_BIT(DDRD, IR_SENSOR_6);
    CLR_BIT(DDRD, IR_SENSOR_7);
    CLR_BIT(DDRD, IR_SENSOR_8);
}

void LED_init(void){
  // Set all to outputs
    SET_BIT(DDRE, LED_0);
    SET_BIT(DDRB, LED_1);
    SET_BIT(DDRB, LED_2);
    SET_BIT(DDRB, LED_3);
    SET_BIT(DDRB, LED_4);
    SET_BIT(DDRD, LED_5);
    SET_BIT(DDRB, LED_6);
    SET_BIT(DDRB, LED_7);
}

void timer_init(void){
    TCCR0A |= (1<<7)|(1<<1)|1;
    TCCR0B |= (1<<2);
}

void motor_init(void){
  
}