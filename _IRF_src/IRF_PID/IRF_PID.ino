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
#define IR_SENSING_THRESHOLD 10

#define CLEAR_MUX ADMUX &= 0b11100000
#define SET_MUX(ADC_CHANNEL) CLEAR_MUX; ADMUX |= ADC_CHANNEL

#define IR1_MUX 0                       // ADC0   // right most
#define IR2_MUX (1<<MUX0)               // ADC1   // second right most
#define IR3_MUX (1<<MUX2)               // ADC4   // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX0)   // ADC5   // middle right center

/* PID Control Coefficients */
#define PROPORTIONAL_COEFFICIENT 1
#define INTEGRAL_COEFFICIENT 0
#define DERIVATIVE_COEFFICIENT 0

/* LEDs */

#define LED_0 PIN6
#define LED_1 PIN0
#define LED_2 PIN1
#define LED_3 PIN2
#define LED_4 PIN7
#define LED_5 PIN0
#define LED_6 PIN6
#define LED_7 PIN5

#define TURN_LED_ON(port, led) port |= (1<<led)
#define TURN_LED_OFF(port, led) port &= ~(1<<led)
#define TOGGLE_LED(port, led) port ^= (1<<led)

/* SERVO motor control */ 
#define SET_SERVO_H(x) OCR1A = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
#define SET_SERVO_V(x) OCR1B = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
#define UPPER_ANGLE_LIMIT ((0.5/20.0) + ((180/180.0) * (2))/20.0) * ICR1
#define LOWER_ANGLE_LIMIT ((0.5/20.0) + ((0/180.0) * (2))/20.0) * ICR1
#define RESET_ANGLE ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1

// #define IR1_MUX (1<<MUX2)                           // ADC4   // right most
// #define IR2_MUX (1<<MUX2) | (1<<MUX0)               // ADC5   // second right most
// #define IR3_MUX (1<<MUX2) | (1<<MUX1)               // ADC6   // right most center
// #define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // ADC7   // middle right center

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  ADC0,   // S1
  ADC1,   // S2
  ADC4,   // S3
  ADC5,   // S4
} IR_STATE_TYPE;

typedef enum {
  IDLE,
  TRACKING,
  HORIZ_LINE,
} IFR_STATE;

////////////////////////////////////////////////
// Function Prototyping
////////////////////////////////////////////////

////////////////////////////////////////////////
// Global Variable Declaration
////////////////////////////////////////////////

// PID Variables
volatile float HP, HI, HD, HLP;
volatile float VP, VI, VD, VLP;
const float Kp = PROPORTIONAL_COEFFICIENT;
const float Ki = INTEGRAL_COEFFICIENT;
const float Kd = DERIVATIVE_COEFFICIENT;
volatile int l_servo_pos, r_servo_pos;
volatile float Herror, Hcorrection, Verror, Vcorrection;
volatile int Hsensor_average, Hsensor_sum, Vsensor_average, Vsensor_sum;
volatile int current_angle_H = 90;
volatile int current_angle_V = 90;
volatile int temp_adc;

// Sensor Array 
// index 0 - TOP LEFT IR SENSOR
// index 1 - TOP RIGHT IR SENSOR
// index 2 - BOTTOM LEFT IR SENSOR
// index 3 - BOTTOM RIGHT IR SENSOR
volatile int sensor_array[4] = {0,0,0,0};
// index 0 - LEFT (-)
// index 1 - RIGHT (+)
volatile int Hsensor_array[2] = {0,0};
// index 0 - DOWN (-)
// index 1 - UP (+)
volatile int Vsensor_array[2] = {0,0};
volatile IR_STATE_TYPE ir_state = ADC4;

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

void setup(){
  cli();

  // timer_init();
  ADC_init();
  SERVO_PWM_init();

  sei();

  Serial.begin(9600);
}

void loop(){
  PID_calc();
  turn_calc();
  Serial.println("Sensor Readings");
  Serial.println("---------------");
  Serial.print("Top Sensor ==> ");
  Serial.println(sensor_array[0]);
  Serial.print("Bottom Sensor ==> ");
  Serial.println(sensor_array[1]);
  Serial.print("Right Sensor ==> ");
  Serial.println(sensor_array[2]);
  Serial.print("Left Sensor ==> ");
  Serial.println(sensor_array[3]);
  Serial.println("---------------");
  Serial.println("Logic Control");
  Serial.println("---------------");
  Serial.print("Horizontal Correction Angle ==> ");
  Serial.println(Hcorrection);
  Serial.print("Horizontal Error (right +ve) ==> ");
  Serial.println(Herror);
  Serial.print("Vertical Correction Angle ==> ");
  Serial.println(Vcorrection);
  Serial.print("Vertical Error (up +ve) ==> ");
  Serial.println(Verror);
  Serial.println("---------------");
  delay(200);
}

////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

void PID_calc(void){
  /* Sensor Conversions */
  // Sensor Array
  // index 0 - TOP LEFT IR SENSOR
  // index 1 - TOP RIGHT IR SENSOR
  // index 2 - BOTTOM LEFT IR SENSOR
  // index 3 - BOTTOM RIGHT IR SENSOR
  Hsensor_array[0] = sensor_array[3]; // LEFT (-)
  Hsensor_array[1] = sensor_array[2]; // RIGHT (+)
  Vsensor_array[0] = sensor_array[1]; // DOWN (-)
  Vsensor_array[1] = sensor_array[0]; // UP (+)

  /* Horizontal Correction Calculations */
  Hsensor_average = 0;
  Hsensor_sum = 0;
  Hsensor_average = (Hsensor_array[0] * -1 * 1000) + (Hsensor_array[1] * 1 * 1000);
  Hsensor_sum = Hsensor_array[0] + Hsensor_array[1];
  Herror = int(Hsensor_average / Hsensor_sum);
  HP = Herror;
  HI += HP;
  HD = HP - HLP;
  HLP = HP;
  Hcorrection = (Kp*HP + Ki*HI + Kd*HD);


  /* Vertical Correction Calculations */
  Vsensor_average = 0;
  Vsensor_sum = 0;
  Vsensor_average = (Vsensor_array[0] * -1 * 1000) + (Vsensor_array[1] * 1 * 1000);
  Vsensor_sum = Vsensor_array[0] + Vsensor_array[1];
  Verror = int(Vsensor_average / Vsensor_sum);
  VP = Verror;  
  VI += VP;
  VD = VP - VLP;
  VLP = VP;
  Vcorrection = (Kp*VP + Ki*VI + Kd*VD);
}

void turn_calc(void){

  current_angle_H += Hcorrection;
  current_angle_V += Vcorrection;

  if (current_angle_H > UPPER_ANGLE_LIMIT){
    current_angle_H = 180;
  } else if (current_angle_H < LOWER_ANGLE_LIMIT) {
    current_angle_H = 0;
  }

  if (current_angle_V > UPPER_ANGLE_LIMIT){
    current_angle_V = 180;
  } else if (current_angle_V < LOWER_ANGLE_LIMIT) {
    current_angle_V = 0;
  }
  set_servos(current_angle_H, current_angle_V);
}

void set_servos(int H , int V){
  SET_SERVO_H(H);
  SET_SERVO_V(V);
}

////////////////////////////////////////////////
// Initialisation Functions
////////////////////////////////////////////////

void SERVO_PWM_init(void){
  DDRB |= (1<<5);
  DDRB |= (1<<6);
  DDRD |= 1;
  
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (0<<WGM10);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | 0b100;
  // TCCR1A = 0b10100011;
  // TCCR1B = 1;
  ICR1 = 1249;
  OCR1A = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
  OCR1B = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
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

////////////////////////////////////////////////
// Interupt Service Routines
////////////////////////////////////////////////

ISR(ADC_vect){
  // set variable based on conversion being completed
  temp_adc = ADCH;
  // if (temp_adc > IR_SENSING_THRESHOLD) {
  if (temp_adc > IR_SENSING_THRESHOLD) {
    temp_adc = temp_adc - IR_SENSING_THRESHOLD;
  } else{
    temp_adc = 0;
  }

  switch (ir_state){ 
    case ADC0: // Rightmost sensor
    sensor_array[3] = temp_adc;  
    SET_MUX(IR2_MUX);
    ir_state = ADC1;
    break;
    
    case ADC1: // Second Rightmost sensor
    sensor_array[1] = temp_adc;
    SET_MUX(IR3_MUX);
    ir_state = ADC4;
    break;
    
    case ADC4: // Rightmost center sensor
    sensor_array[0] = temp_adc;
    // line_sensor_array[3] = temp_adc;
    SET_MUX(IR4_MUX);
    ir_state = ADC5;
    break;
    
    case ADC5: // Second Rightmost center sensor
    sensor_array[2] = temp_adc;
    // line_sensor_array[2] = temp_adc;
    SET_MUX(IR1_MUX);
    ir_state = ADC0;
    break;
    
    default:
      SET_MUX(IR1_MUX);
    break;
  }
  // start conversion again
  ADCSRA |= (1<<ADSC);
}