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

// Current
// #define PROPORTIONAL_COEFFICIENT 1
// #define INTEGRAL_COEFFICIENT 0.00
// #define DERIVATIVE_COEFFICIENT 0

// janky
// #define PROPORTIONAL_COEFFICIENT 1.2
// #define INTEGRAL_COEFFICIENT 0.001
// #define DERIVATIVE_COEFFICIENT 35 

// Slow sometimes goes off course, slight oscillations
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
void PWM_init(void);
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

void setup() {
  // put your setup code here, to run once:
  ADC_init();
  Serial.begin(9600);
}

void loop() {
  Serial.println("--------------------");
  Serial.println(sensor_array[0]);
  Serial.println(sensor_array[1]);
  Serial.println(sensor_array[2]);
  Serial.println(sensor_array[3]);
  Serial.println(sensor_array[4]);
  Serial.println(sensor_array[5]);
  Serial.println(sensor_array[6]);
  Serial.println(sensor_array[7]);
  Serial.println("--------------------");
  delay(100);
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

ISR(ADC_vect){
  // set variable based on conversion being completed
  temp_adc = ADCH;
  // if (temp_adc < WHITE_LINE_THRESHOLD) {
  //   temp_adc = 1;
  // } else{
  //   temp_adc = 0;
  // }
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
    // sensor_array[7] = temp_adc;
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