////////////////////////////////////////////////
// Macros
////////////////////////////////////////////////

#define CLR_BIT(reg, clr_bit) reg &= clr_bit
#define TGL_BIT(reg, tgl_bit) reg ^= tgl_bit    
#define SET_BIT(reg, set_bit) reg |= set_bit

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

// delete eventually
// #define IR_SENSOR_1 PIN4
// #define IR_SENSOR_2 PIN5
// #define IR_SENSOR_3 PIN6
// #define IR_SENSOR_4 PIN7
// #define IR_SENSOR_5 PIN4
// #define IR_SENSOR_6 PIN7
// #define IR_SENSOR_7 PIN6
// #define IR_SENSOR_8 PIN4

// todo: experiment to find out what these values should be 
#define WHITE_LINE_THRESHOLD 120 
#define BLACK_LINE_THRESHOLD 120 
#define GREEN_LIGHT_LEVEL 120 
#define RED_LIGHT_LEVEL 120 

#define DETECT_WHITE(sensor) (sensor > WHITE_LINE_THRESHOLD)
#define DETECT_BLACK(sensor) (sensor < BLACK_LINE_THRESHOLD)

#define STRAIGHT_IR_CONFIG (DETECT_BLACK(sensor_array.CNTR_IR1) && DETECT_WHITE(sensor_array.CNTR_IR2) && DETECT_WHITE(sensor_array.CNTR_IR3) && DETECT_BLACK(sensor_array.CNTR_IR4))

  int CNTR_IR1;
  int CNTR_IR2;
  int CNTR_IR3;
  int CNTR_IR4;
/*
low voltage when sees white
high voltage when sees black
S1 = PF4 = ADC4 = MUX ( 0b000100) 
S2 = PF5 = ADC5 = MUX ( 0b000101) 
S3 = PF6 = ADC6 = MUX ( 0b000110) 
S4 = PF7 = ADC7 = MUX ( 0b000111) 
S5 = PB4 = ADC11 = MUX ( 0b100011)
S6 = PD7 = ADC10 = MUX ( 0b100010)
S7 = PD6 = ADC9 = MUX ( 0b100001) 
S8 = PD4 = ADC8 = MUX ( 0b100000) 
*/
#define IR1_MUX 0b000100 // right most
#define IR2_MUX 0b000101 // second right most
#define IR3_MUX 0b000110 // right most center
#define IR4_MUX 0b000111 // middle right center
#define IR5_MUX 0b100011 // middle left center
#define IR6_MUX 0b100010 // left most center
#define IR7_MUX 0b100001 // second left most
#define IR8_MUX 0b100000 // left most

/* LEDs */

#define LED_0 PIN6
#define LED_1 PIN0
#define LED_2 PIN1
#define LED_3 PIN2
#define LED_4 PIN7
#define LED_5 PIN0
#define LED_6 PIN6
#define LED_7 PIN5

/* Motor pins */ 

#define MOTOR_1_FORWARD 
#define MOTOR_1_REVERSE 
#define MOTOR_2_FORWARD 
#define MOTOR_2_REVERSE 

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  // IDLE,
  STRAIGHT,
  TURNING,
  SLOW_ZONE,
  OBSTACLE,
} BOT_STATE_TYPE;

typedef enum {
  OFF,
  STRAIGHT,
  RIGHT_TURN,
  LEFT_TURN,
  SLOW_ZONE,
} MOTOR_STATE_TYPE;

typedef enum {
  STRAIGHT,
  RIGHT_TURN_DETECT,
  LEFT_TURN_DETECT,
  STOP_DETECT,
  SLOW_ZONE_DETECT,
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

////////////////////////////////////////////////
// Global Variable Declaration
////////////////////////////////////////////////

volatile BOT_STATE_TYPE bot_state = IDLE; 
volatile IR_DATA sensor_array = {0,0,0,0,0,0,0,0};
volatile MOTOR_STATE_TYPE motor_state = OFF;
// volatile IR_STATE_TYPE ir_config = STRAIGHT;

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void){
  // Pause interupts before initialisation
  cli();

  // Initialisation
  timer_init();
  ADC_init();
  PWM_init();
  LED_init();

  // Reset interupts after initialisation
  USBCON=0;
  sei();

  // Robot state machine
  bot_state_machine();
}

////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

void bot_state_machine(void){
  while (1){
    switch (bot_state){
      case IDLE:
      if (1) { // PB pressed 
        bot_state = STRAIGHT;
      }
        break;

      case STRAIGHT:
        // Check that we are going straight

        // Check that a turn hasnt occured
        if ((sensor_array.LEFT_IR1 > WHITE_LINE_THRESHOLD) || (sensor_array.LEFT_IR2 > WHITE_LINE_THRESHOLD)){
          bot_state = TURNING;
        }
        break;

      case TURNING:
        break;

      case SLOW_ZONE:
        break;

      case OBSTACLE:
        break;
        
      default:
        break;
    }
  }
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

  // Start ADC
  ADCSRA |= (1<<ADSC);

  ADMUX |= IR1_MUX;
}

void LED_init(void){
  // Set all to outputs
  SET_BIT(DDRE, LED_0);
  SET_BIT(DDRB, LED_1);
  SET_BIT(DDRB, LED_2);
  SET_BIT(DDRB, LED_3);
}

// Tutorial 3
void PWM_init(void){
  // Set timers to correct configuration
  // todo: Set this up to be more readable ie why we set the bits how we do
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<< WGM00);
  TCCR0B |= (1<<CS02);
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
}

////////////////////////////////////////////////
// Function Definitions
////////////////////////////////////////////////

void 

////////////////////////////////////////////////
// Interupt Service Routines
////////////////////////////////////////////////

// ADC conversion interupt
ISR(ADC_vect){
  // set variable based on conversion being completed
  // todo: make this a state machine
  if (ADMUX & IR1_MUX){
    sensor_array.RIGHT_IR2 = ADCH;
    ADMUX |= IR2_MUX;
  } else if {ADMUX & IR2_MUX}{
    sensor_array.RIGHT_IR1 = ADCH;  
    ADMUX |= IR3_MUX;
  } else if {ADMUX & IR3_MUX}{
    sensor_array.CNTR_IR4 = ADCH;  
    ADMUX |= IR4_MUX;
  } else if {ADMUX & IR4_MUX}{
    sensor_array.CNTR_IR3 = ADCH;   
    ADMUX |= IR5_MUX;
  } else if {ADMUX & IR5_MUX}{
    sensor_array.CNTR_IR2 = ADCH;   
    ADMUX |= IR6_MUX;
  } else if {ADMUX & IR6_MUX}{
    sensor_array.CNTR_IR1 = ADCH;   
    ADMUX |= IR7_MUX;
  } else if {ADMUX & IR7_MUX}{
    sensor_array.LEFT_IR2 = ADCH;  
    ADMUX |= IR8_MUX;
  } else if {ADMUX & IR8_MUX}{
    sensor_array.LEFT_IR1 = ADCH;  
    ADMUX |= IR1_MUX;
  }
  // start conversion again
  ADCSRA |= (1<<ADSC);
}

// 5ms periodic timer interupt

ISR(TIMER1_COMPA_vect){
  switch (motor_state){
    case OFF:
      // todo: if pb pressed 
    break;

    case STRAIGHT:
    break;

    case RIGHT_TURN:
    break;

    case LEFT_TURN:
    break;

    case SLOW_ZONE:
    break;

    case default:
    break;
  }
}
