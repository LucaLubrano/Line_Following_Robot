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

/* IR sensors */
// todo: experiment to find out what these values should be 
#define WHITE_LINE_THRESHOLD 120
#define BLACK_LINE_THRESHOLD 120 
#define GREEN_LIGHT_LEVEL 120 
#define RED_LIGHT_LEVEL 120 

#define DETECT_WHITE(sensor) (sensor > WHITE_LINE_THRESHOLD)
#define DETECT_BLACK(sensor) (sensor < BLACK_LINE_THRESHOLD)
#define DETECT_RED(sensor) (sensor < RED_LIGHT_LEVEL)
#define DETECT_GREEN(sensor) (sensor < GREEN_LIGHT_LEVEL)
#define DETECT_TURNING_MARKER detect_rising_edge(sensor_array[6], sensor_array[7])
#define PB_PRESSED 0 
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
#define IR5_MUX (1<<MUX1) | (1<<MUX0)               // ADC11  // middle left center
#define IR6_MUX (1<<MUX1)                           // ADC10  // left most center
#define IR7_MUX (1<<MUX0)                           // ADC9   // second left most
#define IR8_MUX       

#define TURN_LED_ON(port, pin) port |= pin
#define TURN_LED_OFF(port, pin) port &= ~pin
#define TOGGLE_LED(port, pin) port ^= pin
/* Motor pins */ 

// check these values through testing
#define STRAIGHT_SPEED 128
#define SLOW_ZONE_SPEED 64
#define TURN_SPEED 96
#define TURN_SPEED_INCREMENT 2
#define TRACK_ADJUSTMENT_SPEED 1

#define SPEED_LIMIT (64+64)
#define SLOW_ZONE_MTR_SPEED 64

#define LEFT_MOTOR OCR0B
#define RIGHT_MOTOR OCR0A 

/* PID Control Coefficients */
#define PROPORTIONAL_COEFFICIENT 22
#define INTEGRAL_COEFFICIENT 0.0001
#define DERIVATIVE_COEFFICIENT 6


// P > D
// D about 1/4 
// start with P go until it becomes unstable (use 30% of result)
// then use D not I 
// use I 

/* Light Setup */
#define STOP_LED_DDR DDRB
#define STRAIGHT_LED_DDR DDRB
#define OBSTACLE_LED_DDR DDRD
#define TURRET_ON_DDR DDRD
#define TURRET_TRACKING_DDR DDRD

#define STOP_LED_PORT PORTB
#define STRAIGHT_LED_PORT PORTB
#define OBSTACLE_LED_PORT PORTD
#define TURRET_ON_PORT PORTD
#define TURRET_TRACKING_PORT PORTD

#define STOP_LED_PIN PIN3
#define STRAIGHT_LED_PIN PIN7
#define OBSTACLE_LED_PIN PIN0
#define TURRET_ON_PIN PIN1
#define TURRET_TRACKING_PIN PIN2

// Recieve signal from turret
#define TURRET_INPUT_DDR 0 
#define TURRET_INPUT_PORT 0
#define TURRET_INPUT_PIN 0

// Send signal to turret
#define TURRET_OUTPUT_DDR 0
#define TURRET_OUTPUT_PORT 0
#define TURRET_OUTPUT_PIN 0

// 
#define SEND_SIGNAL_TO_TURRET SETBIT(TURRET_OUTPUT_PORT, TURRET_OUTPUT_PIN)
#define STOP_SIGNAL_TO_TURRET CLRBIT(TURRET_OUTPUT_PORT, TURRET_OUTPUT_PIN)
#define CHECK_SIGNAL_RECEIVE (TURRET_INPUT_PORT & TURRET_INPUT_PIN)

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  IDLE,
  STRAIGHT,
  TURNING,
  SLOW_STRAIGHT,
  SLOW_TURNING,
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
void bot_state_machine(void);
void motor_state_machine(void);
void IR_state_machine(void);
void PID_calc(void);
void PID_init(void);
void turn_calc(void);
void set_motors(int L , int R);
void COMMS_init(void);
void LED_init(void);
void CLEAR_LEDS(void);
void SET1_LED(int PORT, int PIN);
void SET2_LED(int PORT1, int PIN_1, int PORT2, int PIN_2);
void SET3_LED(int PORT1, int PIN_1, int PORT2, int PIN_2, int PORT3, int PIN_3);
int detect_rising_edge(int current_sensor1, int current_sensor2);

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
const int maxspeed = 128;
const int basespeed = 128;
volatile int r_mtr_speed, l_mtr_speed;
volatile float error, correction, sp;
volatile int position, sensor_average, sensor_sum;
volatile int currentspeedR = 64;
volatile int currentspeedL = 64;
volatile bool slow_zone = false;
volatile int prev_sensor1 = 0;
volatile int prev_sensor2 = 0;
// volatile int current_corner_sensor = 0;


////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

void setup() {
  // Pause interupts before initialisation
  cli();

  // Initialisation
  ADC_init();
  MOTOR_PWM_init();
  COMMS_init();
  LED_init();
  // Reset interupts after initialisation
  sei();
  SET1_LED(STOP_LED_PORT, STOP_LED_PIN);
  set_motors(0,0);
}

void loop() {
  // PID_calc();
  // turn_calc();
  robot_state_machine();
}
////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

void robot_state_machine(void){

  switch(bot_state){
    case IDLE:
      if(PB_PRESSED){
        SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
        bot_state = STRAIGHT;
      }
    break;
    
    case STRAIGHT:
      PID_calc();
      turn_calc();
      if(DETECT_TURNING_MARKER){
        CLEAR_LEDS();
        bot_state = TURNING;
      }
      if(DETECT_RED){
        SEND_SIGNAL_TO_TURRET;
        SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN);
        bot_state = SLOW_ZONE_STRAIGHT;
        set_motors(SLOW_ZONE_MTR_SPEED, SLOW_ZONE_MTR_SPEED);
      } 
    break;

    case TURNING:
      PID_calc();
      turn_calc();
      if(DETECT_TURNING_MARKER){
        SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
        bot_state = STRAIGHT;
      }
      if(DETECT_RED){
        SEND_SIGNAL_TO_TURRET;
        SET1_LED(TURRET_ON_PORT, TURRET_ON_PIN);
        bot_state = SLOW_ZONE_TURNING;
        set_motors(SLOW_ZONE_MTR_SPEED, SLOW_ZONE_MTR_SPEED);
      } 
    break;

    case SLOW_ZONE_STRAIGHT:
      PID_calc();
      turn_calc();

      if(DETECT_TURNING_MARKER){
        if(RECEIVE_TRACKING_SIGNAL){
          SET2_LED(TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else{
          SET1_LED(TURRET_ON_PORT, TURRET_ON_PIN);
        }
        bot_state = SLOW_ZONE_TURNING;
      } else if(DETECT_GREEN){
        STOP_SIGNAL_TO_TURRET;
        bot_state = STRAIGHT;
        SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
        set_motors(SPEED_LIMIT, SPEED_LIMIT);
      } 
    
      if(DETECT_OBSTACLE){
        set_motors(OBSTACLE_SPEED, OBSTACLE_SPEED);
        SET4_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN, OBSTACLE_LED_PORT, OBSTACLE_LED_PIN);
      } else {
        if(CHECK_SIGNAL_RECEIVE){
          SET3_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else {
          SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        }
      }
    break;

    case SLOW_ZONE_TURNING:
      PID_calc();
      turn_calc();

      if(DETECT_TURNING_MARKER){
        if(CHECK_SIGNAL_RECEIVE){
          SET3_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else{
          SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN);
        }
        bot_state = SLOW_ZONE_STRAIGHT;
      } else if(DETECT_GREEN){
        STOP_SIGNAL_TO_TURRET;
        bot_state = TURNING;
        CLEAR_LEDS();
        set_motors(SPEED_LIMIT, SPEED_LIMIT);
      } 
    
    break;
  }





  // switch(bot_state){
  //   case IDLE:      
  //     if (PB_PRESSED){
  //       SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
  //       bot_state = STRAIGHT;
  //     }
  //   break;

  //   case STRAIGHT:
  //     PID_calc();
  //     turn_calc();

  //     // Check if we are going around a corner
  //     if (DETECT_CORNER(sensor_array[6], sensor_array[7])){
  //       // If not in a slowzone, we want no lights on
  //       if(!slow_zone){
  //         CLEAR_LEDS();  
  //       } else if (slow_zone) { // If in a slow zone want turret light on
  //         SET1_LED(TURRET_ON_PORT, STRAIGHT_LED_PIN);
  //       }
  //       bot_state = TURNING;    
  //     } else {
  //       // Check if we are entering of exiting a slow zone
  //       if (DETECT_RED(0)){
  //         slow_zone = true;
  //         SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN);
  //         SEND_SIGNAL_TO_TURRET;
  //         bot_state = SLOW_ZONE;    
  //       } else if (DETECT_GREEN(0)){
  //         slow_zone = false;
  //         SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
  //         STOP_SIGNAL_TO_TURRET;
  //         bot_state = STRAIGHT;
  //       }
  //     }
      
  //   break;

  //   case TURNING:
  //     PID_calc();
  //     turn_calc();

  //     // Check if we are coming out of the corner
  //     if (DETECT_CORNER(sensor_array[6], sensor_array[7])){
  //       SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
  //       bot_state = STRAIGHT;    
  //     } else if (DETECT_RED(0)){
  //       TURN_LED_OFF(DDRB, TURN_LED);
  //       TURN_LED_ON(DDRE, SLOW_LED);
  //       bot_state = SLOW_ZONE;    
  //     }
  //   break;

  //   case SLOW_ZONE:
  //     PID_calc();
  //     turn_calc();
  //     if(DETECT_GREEN(0)){
  //       TURN_LED_OFF(DDRB, SLOW_LED);
  //       bot_state = STRAIGHT;
  //     }
  //     PID_calc();
  //     turn_calc();
  //   break;

  //   case OBSTACLE:
      
    
  //   bot_state = STRAIGHT;
  //   break;

  //   default:
  //   bot_state = IDLE;
  //   break;
  // }
}


// todo: make the pwm init run when a button is pressed
void sensor_value_average(void){
  sensor_average = 0;
  sensor_sum = 0;
  // todo: (?) make a for loop?
  sensor_average = (line_sensor_array[0] * -2 * 1000.0) + (line_sensor_array[1] * -1 * 1000) + (line_sensor_array[2] * 1 * 1000) + (line_sensor_array[3] * 2 * 1000);
  // sensor_average = (line_sensor_array[0] * -2) + (line_sensor_array[1] * -1) + (line_sensor_array[2] * 1) + (line_sensor_array[3] * 2);
  sensor_sum = int(line_sensor_array[0]) + int(line_sensor_array[1]) + int(line_sensor_array[2]) + int(line_sensor_array[3]);
  position = sensor_average / sensor_sum;
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
//   r_mtr_speed = basespeed + correction;
//   l_mtr_speed = basespeed - correction;

// if (r_mtr_speed > maxspeed){
//     r_mtr_speed = maxspeed;
//   } else if (r_mtr_speed < 0) {
//     r_mtr_speed = 0;
//   }

//   if (l_mtr_speed > maxspeed) {
//     l_mtr_speed = maxspeed;
//   } else if (l_mtr_speed < 0){
//     l_mtr_speed = 0;
//   }
//   set_motors(l_mtr_speed, r_mtr_speed);

  currentspeedR += correction;
  currentspeedL -= correction;

  if (currentspeedR > maxspeed){
    currentspeedR = maxspeed;
  } else if (currentspeedR < 0) {
    currentspeedR = 0;
  }

  if (currentspeedL > maxspeed) {
    currentspeedL = maxspeed;
  } else if (currentspeedL < 0){
    currentspeedL = 0;
  }

  if (correction != 0){ 
    set_motors(currentspeedL, currentspeedR);
  }
  
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
void COMMS_init(void){
  SETBIT(TURRET_INPUT_DDR, TURRET_INPUT_PIN); // Set SIG TRK to input
  SETBIT(TURRET_OUTPUT_DDR, TURRET_OUTPUT_PIN); // Set SIG SZ to input
  CLRBIT(TURRET_INPUT_PORT, TURRET_INPUT_PIN); // Set to low
  CLRBIT(TURRET_OUTPUT_PORT, TURRET_OUTPUT_PIN); // Set to low
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
void LED_init(void){
  // Set as output
  SETBIT(STOP_LED_DDR, STOP_LED_PIN);
  SETBIT(STRAIGHT_LED_DDR, STRAIGHT_LED_PIN);
  SETBIT(OBSTACLE_LED_DDR, OBSTACLE_LED_PIN);
  SETBIT(TURRET_ON_DDR, TURRET_ON_PIN);
  SETBIT(TURRET_TRACKING_DDR, TURRET_TRACKING_PIN);
  CLEAR_LEDS();
}
void CLEAR_LEDS(void){
  // Set to off
  CLRBIT(STOP_LED_PORT, STOP_LED_PIN);
  CLRBIT(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
  CLRBIT(OBSTACLE_LED_PORT, OBSTACLE_LED_PIN);
  CLRBIT(TURRET_ON_PORT, TURRET_ON_PIN);
  CLRBIT(TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
}
void SET1_LED(int PORT, int PIN){
  CLEAR_LEDS();
  SETBIT(PORT, PIN);
}
void SET2_LED(int PORT1, int PIN_1, int PORT2, int PIN_2){
  CLEAR_LEDS();
  SETBIT(PORT1, PIN_1);
  SETBIT(PORT2, PIN_2);
}
void SET3_LED(int PORT1, int PIN_1, int PORT2, int PIN_2, int PORT3, int PIN_3){
  CLEAR_LEDS();
  SETBIT(PORT1, PIN_1);
  SETBIT(PORT2, PIN_2);
  SETBIT(PORT3, PIN_3);
}
void SET4_LED(int PORT1, int PIN_1, int PORT2, int PIN_2, int PORT3, int PIN_3, int PORT4, int PIN_3){
  CLEAR_LEDS();
  SETBIT(PORT1, PIN_1);
  SETBIT(PORT2, PIN_2);
  SETBIT(PORT3, PIN_3);
  SETBIT(PORT4, PIN_4);
}
int detect_rising_edge(int current_sensor1, int current_sensor2){
  int val_changed = 0; int rising_edge = 0;
  val_changed = (prev_sensor1 ^ current_sensor1) | (prev_sensor2 ^ current_sensor2);
  rising_edge = (prev_sensor1 & current_sensor1) | (prev_sensor2 & current_sensor2);
  return rising_edge;
} 

////////////////////////////////////////////////
// Interupt Service Routines
////////////////////////////////////////////////

// ADC conversion interupt
ISR(ADC_vect){

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
    // line_sensor_array[3] = temp_adc;
    SET_MUX(IR4_MUX);
    ir_state = ADC7;
    break;
    
    case ADC7: // Second Rightmost center sensor
    sensor_array[3] = temp_adc;
    line_sensor_array[3] = temp_adc;
    SET_MUX(IR5_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC11;
    break;
    
    case ADC11: // Second Leftmost center sensor
    sensor_array[4] = temp_adc;
    line_sensor_array[2] = temp_adc;
    SET_MUX(IR6_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC10;
    break;
    
    case ADC10: // Leftmost center sensor
    sensor_array[5] = temp_adc;
    // line_sensor_array[0] = temp_adc;
    SET_MUX(IR7_MUX);
    ADCSRB |= (1<<MUX5);
    ir_state = ADC9;
    break;
    
    case ADC9: // second Leftmost sensor
    sensor_array[6] = temp_adc;
    line_sensor_array[1] = temp_adc;
    CLEAR_MUX;
    ADCSRB |= (1<<MUX5);
    ir_state = ADC8;
    break;
    
    case ADC8: // Leftmost sensor
    sensor_array[7] = temp_adc;
    line_sensor_array[0] = temp_adc;
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

