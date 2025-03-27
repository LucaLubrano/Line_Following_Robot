// PID control reference
// https://www.teachmemicro.com/implementing-pid-for-a-line-follower-robot/

////////////////////////////////////////////////
// Macros
////////////////////////////////////////////////

#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
#define TGLBIT(x,y) x ^= BIT(y)

////////////////////////////////////////////////
// Configurations
////////////////////////////////////////////////

/* General Pin Arrangement */

#define PIN0 0b00000001
#define PIN1 0b00000010
#define PIN2 0b00000100 
#define PIN3 0b00001000
#define PIN4 0b00010000
#define PIN5 0b00100000
#define PIN6 0b01000000
#define PIN7 0b10000000

/* IR sensors */

// todo: experiment to find out what these values should be 
#define WHITE_LINE_THRESHOLD 120
#define BLACK_LINE_THRESHOLD 120 
#define GREEN_LIGHT_LEVEL 120 
#define RED_LIGHT_LEVEL 120 

#define DETECT_WHITE(sensor) (sensor > WHITE_LINE_THRESHOLD)
#define DETECT_BLACK(sensor) (sensor < BLACK_LINE_THRESHOLD)

// #define STRAIGHT_IR_CONFIG (DETECT_BLACK(sensor_array.CNTR_IR1) && DETECT_WHITE(sensor_array.CNTR_IR2) && DETECT_WHITE(sensor_array.CNTR_IR3) && DETECT_BLACK(sensor_array.CNTR_IR4))

#define LEFT_SENSORS_DETECT_WHITE ((sensor_array.LEFT_IR1 > WHITE_LINE_THRESHOLD) || (sensor_array.LEFT_IR2 > WHITE_LINE_THRESHOLD))
#define RIGHT_SENSORS_DETECT_WHITE ((sensor_array.RIGHT_IR1 > WHITE_LINE_THRESHOLD) || (sensor_array.RIGHT_IR2 > WHITE_LINE_THRESHOLD))
#define LEFT_LINE_OFF_COURSE (sensor_array.CNTR_IR1 > WHITE_LINE_THRESHOLD)
#define RIGHT_LINE_OFF_COURSE (sensor_array.CNTR_IR4 > WHITE_LINE_THRESHOLD)

#define CLEAR_MUX ADMUX &= 0b11100000
#define SET_MUX(ADC_CHANNEL) CLEAR_MUX; ADMUX |= ADC_CHANNEL

#define IR1_MUX (1<<MUX2)                           // ADC4   // right most
#define IR2_MUX (1<<MUX2) | (1<<MUX0)               // ADC5   // second right most
#define IR3_MUX (1<<MUX2) | (1<<MUX1)               // ADC6   // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // ADC7   // middle right center
#define IR5_MUX (1<<MUX1) | (1<<MUX0)   // ADC11  // middle left center
#define IR6_MUX (1<<MUX1)               // ADC10  // left most center
#define IR7_MUX (1<<MUX0)               // ADC9   // second left most
#define IR8_MUX                            // ADC8   // left most

/* LEDs */

#define LED_0 PIN6
#define LED_1 PIN0
#define LED_2 PIN1
#define LED_3 PIN2
#define LED_4 PIN7
#define LED_5 PIN0
#define LED_6 PIN6
#define LED_7 PIN5

#define TURN_LED_ON(direction_register, led) direction_register |= (1<<led)
#define TURN_LED_OFF(direction_register, led) direction_register &= ~(1<<led)

/* Motor pins */ 

// check these values through testing
#define STRAIGHT_SPEED 128
#define SLOW_ZONE_SPEED 64
#define TURN_SPEED 96
#define TURN_SPEED_INCREMENT 2
#define TRACK_ADJUSTMENT_SPEED 1

#define SPEED_LIMIT (64+64)

#define LEFT_MOTOR OCR0B
#define RIGHT_MOTOR OCR0A 

/* PID Control Coefficients */
#define PROPORTIONAL_COEFFICIENT 3
#define INTEGRAL_COEFFICIENT 0
#define DERIVATIVE_COEFFICIENT 1


// P > D
// D about 1/4 
// start with P go until it becomes unstable (use 30% of result)
// then use D not I 
// use I 

/* Buttons */
#define PB_PRESSED (PINC & (1<<7))

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  IDLE,
  STRAIGHT,
  TURNING,
  SLOW_ZONE,
  OBSTACLE,
} BOT_STATE_TYPE;

typedef enum {
  MTR_OFF,
  MTR_STRAIGHT,
  MTR_RIGHT_TURN,
  MTR_LEFT_TURN,
  MTR_SLOW_ZONE,
} MOTOR_STATE_TYPE;

typedef enum {
  ADC4,   // S1
  ADC5,   // S2
  ADC6,   // S3
  ADC7,   // S4
  ADC11,  // S5
  ADC10,  // S6
  ADC9,   // S7
  ADC8,   // S8
} IR_STATE_TYPE;

typedef struct {
  int CNTR_IR1;
  int CNTR_IR2;
  int CNTR_IR3;
  int CNTR_IR4;
  int LEFT_IR1;
  int LEFT_IR2;
  int RIGHT_IR1;
  int RIGHT_IR2;
} IR_DATA;

////////////////////////////////////////////////
// Function Prototyping
////////////////////////////////////////////////

void timer_init(void);
void ADC_init(void);
void MOTOR_PWM_init(void);
void LED_init(void);
void bot_state_machine(void);
void motor_state_machine(void);
void IR_state_machine(void);
void PID_calc(void);
void PID_init(void);
void turn_calc(void);
void set_motors(int L , int R);

////////////////////////////////////////////////
// Global Variable Declaration
////////////////////////////////////////////////

volatile BOT_STATE_TYPE bot_state = STRAIGHT; 
volatile int sensor_array[8] = {0,0,0,0,0,0,0,0};
volatile int line_sensor_array[4] = {0,0,0,0};
volatile MOTOR_STATE_TYPE motor_state = MTR_OFF;
volatile IR_STATE_TYPE ir_state = ADC4;

volatile int temp_adc = 0;

// PID Variables
volatile float P, I, D, LP;
const float Kp = PROPORTIONAL_COEFFICIENT;
const float Ki = INTEGRAL_COEFFICIENT;
const float Kd = DERIVATIVE_COEFFICIENT;
const int base_speed = 64;
volatile int r_mtr_speed, l_mtr_speed;
volatile float error, correction, sp;
volatile int position, sensor_average, sensor_sum;

// State machine
static bool bot_turning = false;

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void){
  // Pause interupts before initialisation
  cli();

  // Initialisation
  timer_init();
  ADC_init();
  MOTOR_PWM_init();
  LED_init();

  // Reset interupts after initialisation
  USBCON=0;
  sei();

  PID_init();
  // Robot state machine
  // bot_state_machine();
  while(1){
    PID_calc();
    turn_calc();
  }
}

////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

// todo: make the pwm init run when a button is pressed
void sensor_value_average(void){
  sensor_average = 0;
  sensor_sum = 0;
  // todo: (?) make a for loop?
  sensor_average = (line_sensor_array[0] * -2 * 1000) + (line_sensor_array[1] * -1 * 1000) + (line_sensor_array[2] * 1 * 1000) + (line_sensor_array[3] * 2 * 1000);
  // sensor_average = (line_sensor_array[0] * -2) + (line_sensor_array[1] * -1) + (line_sensor_array[2] * 1) + (line_sensor_array[3] * 2);
  sensor_sum = int(line_sensor_array[0]) + int(line_sensor_array[1]) + int(line_sensor_array[2]) + int(line_sensor_array[3]);
  position = int(sensor_average / sensor_sum);
}

// Check if this is needed
void PID_init(void){
  sensor_value_average();
  sp = position;
}

void PID_calc(void){
  sensor_value_average();
  // error = position - sp;
  error = position ;
  P = error;
  I += P;
  D = P - LP;
  LP = P;
  correction = int(Kp*P + Ki*I + Kd*D);

}

void turn_calc(void){
  r_mtr_speed = base_speed + correction;
  l_mtr_speed = base_speed - correction;

if (r_mtr_speed > SPEED_LIMIT){
    r_mtr_speed = base_speed;
  } else if (r_mtr_speed < 0) {
    r_mtr_speed = 0;
  }

  if (l_mtr_speed > SPEED_LIMIT) {
    l_mtr_speed = base_speed;
  } else if (l_mtr_speed < 0){
    l_mtr_speed = 0;
  }
  set_motors(l_mtr_speed, r_mtr_speed);
}

void set_motors(int L , int R){
  
  RIGHT_MOTOR = R;
  LEFT_MOTOR = L;

}

////////////////////////////////////////////////
// Initialisation Functions
////////////////////////////////////////////////

void timer_init(void){
  // 5ms Periodic Timer
  // todo: pretty this section up / add description
  TCCR1A = 0;
  TCCR1B |= (1<<CS12);
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 31250;
  TIMSK1 = 2;
}

void ADC_init(void){
  // Set up ADC
  ADMUX |= (1<<REFS0) | (1<<ADLAR);
  ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB |= (1<<ADHSM);

  // set first conversion
  SET_MUX(IR1_MUX);

  // Start ADC
  ADCSRA |= (1<<ADSC);  
}

void LED_init(void){
  // Set all to outputs
  SETBIT(DDRE, LED_0);
  SETBIT(DDRB, LED_1);
  SETBIT(DDRB, LED_2);
  SETBIT(DDRB, LED_3);
}

void MOTOR_PWM_init(void){
  // Set pins b7 and d0 to output to expose pwm signal to pin
  SETBIT(DDRB, PB7);
  SETBIT(DDRD, PD0);
  
  // todo: Set this up to be more readable ie why we set the bits how we do
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
  TCCR0B |= (1<<CS02);

  // set pits b0 and e6 to outputs for phase control
  // SETBIT(DDRB, PB0);
  // SETBIT(DDRE, PE6);

  // // set output direction values
  // SETBIT(PORTB, PB0);
  // CLRBIT(PORTE, PE6);
}

// switch 1
void button_init(void){ 
  DDRC &= ~(1<<7);
  PORTB &= ~(1<<7);
}

////////////////////////////////////////////////
// Function Definitions
////////////////////////////////////////////////

////////////////////////////////////////////////
// Interupt Service Routines
////////////////////////////////////////////////

// ADC conversion interupt
// todo: MUX 5 is in ADCSRB register CHECK
ISR(ADC_vect){
  // set variable based on conversion being completed
  temp_adc = ADCH;
  if (temp_adc < WHITE_LINE_THRESHOLD) {
    temp_adc = 1;
  } else{
    temp_adc = 0;
  }
  switch (ir_state){ 
    case ADC4: // Rightmost sensor
    sensor_array[0] = temp_adc;  
    SET_MUX(IR2_MUX);
    ADCSRB &= ~(1<<MUX5);
    ir_state = ADC5;
    break;
    
    case ADC5: // Second Rightmost sensor
    sensor_array[1] = temp_adc;
    SET_MUX(IR3_MUX);
    ir_state = ADC6;
    break;
    
    case ADC6: // Rightmost center sensor
    sensor_array[2] = temp_adc;
    line_sensor_array[3] = temp_adc;
    SET_MUX(IR4_MUX);
    ir_state = ADC7;
    break;
    
    case ADC7: // Second Rightmost center sensor
    sensor_array[3] = temp_adc;
    line_sensor_array[2] = temp_adc;
    SET_MUX(IR5_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC11;
    break;
    
    case ADC11: // Second Leftmost center sensor
    sensor_array[4] = temp_adc;
    line_sensor_array[1] = temp_adc;
    SET_MUX(IR6_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC10;
    break;
    
    case ADC10: // Leftmost center sensor
    sensor_array[5] = temp_adc;
    line_sensor_array[0] = temp_adc;
    SET_MUX(IR7_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC9;
    break;
    
    case ADC9: // second Leftmost sensor
    sensor_array[6] = temp_adc;
    CLEAR_MUX;
    ADCSRB |= (1<<MUX5);
    ir_state = ADC8;
    break;
    
    case ADC8: // Leftmost sensor
    sensor_array[7] = temp_adc;
    SET_MUX(IR1_MUX);
    ADCSRB &= ~(1<<MUX5);
    ir_state = ADC4;
    break;
    
    default:
      SET_MUX(IR1_MUX);
    break;
  }
  // start conversion again
  ADCSRA |= (1<<ADSC);
}

// 5ms periodic timer interupt

// ISR(TIMER1_COMPA_vect){
//   switch(bot_state){
//     case IDLE:
//       if PB_PRESSED{
//         TURN_LED_ON(DDRE, LED_0);
//         bot_state = STRAIGHT;
//       }
//     break;

//     case STRAIGHT:
//       if (LEFT_SENSORS_DETECT_WHITE || RIGHT_SENSORS_DETECT_WHITE){
//         TURN_LED_OFF(DDRE, LED_0);
//         TURN_LED_ON(DDRB, LED_1);
//         bot_state = TURNING;    
//       }

//       if (DETECT_RED){
//         bot_state = SLOW_ZONE;    
//       }
//     break;

//     case TURNING:
//       if (LEFT_SENSORS_DETECT_WHITE || RIGHT_SENSORS_DETECT_WHITE){
//         TURN_LED_ON(DDRE, LED_0);
//         TURN_LED_OFF(DDRB, LED_1);
//         bot_state = STRAIGHT;    
//       }

//       bot_state = ;
//       if (DETECT_RED){
//         bot_state = SLOW_ZONE;    
//       }
//     break;

//     case SLOW_ZONE:
//       if(DETECT_GREEN){
//         bot_state = TURNING;
//       }
//       bot_state = STRAIGHT;
//     break;

//     case OBSTACLE:
    
//     bot_state = STRAIGHT;
//     break;

//     default:
//     bot_state = IDLE;
//     break;
//   }
// }
