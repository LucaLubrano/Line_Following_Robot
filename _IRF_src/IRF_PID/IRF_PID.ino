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

#define CLEAR_MUX ADMUX &= 0b11100000
#define SET_MUX(ADC_CHANNEL) CLEAR_MUX; ADMUX |= ADC_CHANNEL

#define IR1_MUX (1<<MUX2)                           // ADC4   // right most
#define IR2_MUX (1<<MUX2) | (1<<MUX0)               // ADC5   // second right most
#define IR3_MUX (1<<MUX2) | (1<<MUX1)               // ADC6   // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // ADC7   // middle right center

/* PID Control Coefficients */
#define PROPORTIONAL_COEFFICIENT 3
#define INTEGRAL_COEFFICIENT 0
#define DERIVATIVE_COEFFICIENT 1

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



////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  ADC4,   // S1
  ADC5,   // S2
  ADC6,   // S3
  ADC7,   // S4
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

// Sensor Array 
// index 0 - TOP LEFT IR SENSOR
// index 1 - TOP RIGHT IR SENSOR
// index 2 - BOTTOM LEFT IR SENSOR
// index 3 - BOTTOM RIGHT IR SENSOR
volatile int sensor_array[4] = {0,0,0,0};
volatile int Hsensor_array[4] = {0,0};
volatile int Vsensor_array[4] = {0,0};

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void){
  TCCR1A = 0b10100011 ;
  TCCR1B = 1;
  DDRB |= 1<<7;
  DDRD |= 1;
  ICR1 = 1000; 
  OCR1A = 1/20 * ICR1;

  // OCR1A = int(1/20 * 1249);
  while(1);
    // OCR1A += int(1/20 * 1249);
}

////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

void PID_calc(void){
  /* Sensor Conversions */
  Hsensor_array[0] = sensor_array[0] + sensor_array[2]; // LEFT (-)
  Hsensor_array[1] = sensor_array[1] + sensor_array[3]; // RIGHT (+)
  Vsensor_array[0] = sensor_array[2] + sensor_array[3]; // DOWN (-)
  Vsensor_array[1] = sensor_array[2] + sensor_array[3]; // UP (+)

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
  Hcorrection = int(Kp*HP + Ki*HI + Kd*HD);


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
  Vcorrection = int(Kp*VP + Ki*VI + Kd*VD);
}

void turn_calc(void){
  r_servo_pos = base_angle + correction;
  l_servo_pos = base_angle - correction;

if (r_servo_pos > SPEED_LIMIT){
    r_servo_pos = base_angle;
  } else if (r_servo_pos < 0) {
    r_servo_pos = 0;
  }

  if (l_servo_pos > SPEED_LIMIT) {
    l_servo_pos = base_angle;
  } else if (l_servo_pos < 0){
    l_servo_pos = 0;
  }
  set_servos(l_servo_pos, r_servo_pos);
}

void set_servos(int H , int V){
  
  HORIZONTAL_SERVO = H;
  VERTICAL_SERVO = V;

}

////////////////////////////////////////////////
// Initialisation Functions
////////////////////////////////////////////////

void SERVO_PWM_init(void){
  // Check that this code works
  TCCR1A = 0b10100011 ;
  TCCR1B = 1;
  DDRB |= 1<<7;
  DDRD |= 1;
  ICR1 = 1000; // check this value
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
  if (temp_adc < WHITE_LINE_THRESHOLD) {
    temp_adc = 1;
  } else{
    temp_adc = 0;
  }

  switch (ir_state){ 
    case ADC4: // Rightmost sensor
    sensor_array[0] = temp_adc;  
    SET_MUX(IR2_MUX);
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
    ir_state = ADC4;
    break;
    
    default:
      SET_MUX(IR1_MUX);
    break;
  }
  // start conversion again
  ADCSRA |= (1<<ADSC);
}