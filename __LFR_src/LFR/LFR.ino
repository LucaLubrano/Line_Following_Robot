// PID controll tutorial
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

#define IR1_MUX (1<<MUX2)                           // right most
#define IR2_MUX (1<<MUX2) | (1<<MUX0)               // second right most
#define IR3_MUX (1<<MUX2) | (1<<MUX1)               // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // middle right center
#define IR5_MUX (1<<MUX5) | (1<<MUX1) | (1<<MUX0)   // middle left center
#define IR6_MUX (1<<MUX5) | (1<<MUX1)               // left most center
#define IR7_MUX (1<<MUX5) | (1<<MUX0)               // second left most
#define IR8_MUX (1<<MUX5)                           // left most

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

// check these values through testing
#define STRAIGHT_SPEED 128
#define SLOW_ZONE_SPEED 64
#define TURN_SPEED 96
#define TURN_SPEED_INCREMENT 2
#define TRACK_ADJUSTMENT_SPEED 1

#define LEFT_MOTOR OCR0B
#define RIGHT_MOTOR OCR0A 

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  IDLE,
  STRAIGHT,
  TURNING,
  RIGHT_TURN,
  LEFT_TURN,
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
  ADC4,
  ADC5,
  ADC6,
  ADC7,
  ADC11,
  ADC10,
  ADC9,
  ADC8,
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

volatile BOT_STATE_TYPE bot_state = STRAIGHT; 
volatile IR_DATA sensor_array = {0,0,0,0,0,0,0,0};
volatile MOTOR_STATE_TYPE motor_state = MTR_OFF;
volatile IR_STATE_TYPE ir_state = ADC4;

volatile int temp_adc = 0;

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
        RIGHT_MOTOR = STRAIGHT_SPEED;
        LEFT_MOTOR = STRAIGHT_SPEED;

        // Check that a turn indicator hasnt been detected
        if (LEFT_SENSORS_DETECT_WHITE){
          bot_state = RIGHT_TURN;
          RIGHT_MOTOR = TURN_SPEED;
          LEFT_MOTOR = TURN_SPEED;
        } else if (RIGHT_SENSORS_DETECT_WHITE){
          bot_state = LEFT_TURN;
          RIGHT_MOTOR = TURN_SPEED;
          LEFT_MOTOR = TURN_SPEED;
        }

        // Check we are still on the track
        if (LEFT_LINE_OFF_COURSE) {
          RIGHT_MOTOR += TRACK_ADJUSTMENT_SPEED;
        } else if (RIGHT_LINE_OFF_COURSE) {

        }

        break;

      case RIGHT_TURN:
        
        // Check that we are matching the turn
        if RIGHT_LINE_OFF_COURSE {
          LEFT_MOTOR += TURN_SPEED_INCREMENT;
        }
        // Check that the turn hasnt ended
        if LEFT_SENSORS_DETECT_WHITE{
          bot_state = STRAIGHT;
          // change motor speed?
        }

        break;

      case LEFT_TURN:
        // Check that we are matching the turn
        if LEFT_LINE_OFF_COURSE {
          RIGHT_MOTOR += TURN_SPEED_INCREMENT;
        }
        // Check that the turn hasnt ended
        if RIGHT_SENSORS_DETECT_WHITE{
          bot_state = STRAIGHT;
          // change motor speed?
        }

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

  ADMUX |= (1<<MUX0);

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

void PWM_init(void){
  // Set timers to correct configuration
  // todo: Set this up to be more readable ie why we set the bits how we do
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<< WGM00);
  TCCR0B |= (1<<CS02);
  
  // Set pins b7 and d0 to output to expose pwm signal to pin
  SETBIT(DDRB, PB7);
  SETBIT(DDRD, PD0);

  // set pits b0 and e6 to outputs for phase control
  SETBIT(DDRB, PB0);
  SETBIT(DDRE, PE6);

  // set output direction values
  SETBIT(PORTB, PB0);
  CLRBIT(PORTE, PE6);
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
ISR(ADC_vect){
  // set variable based on conversion being completed
  temp_adc = ADCH;
  switch (ir_state){ 
    case ADC4: // Rightmost sensor
    sensor_array.RIGHT_IR2 = temp_adc;
    SET_MUX(IR2_MUX);
    ir_state = ADC5;
    break;
    
    case ADC5: // Second Rightmost sensor
    sensor_array.RIGHT_IR1 = temp_adc;
    SET_MUX(IR3_MUX);
    ir_state = ADC6;
    break;
    
    case ADC6: // Rightmost center sensor
    sensor_array.CNTR_IR4 = temp_adc;
    SET_MUX(IR4_MUX);
    ir_state = ADC7;
    break;
    
    case ADC7: // Second Rightmost center sensor
    sensor_array.CNTR_IR3 = temp_adc;
    SET_MUX(IR5_MUX);
    ir_state = ADC11;
    break;
    
    case ADC11: // Second Leftmost center sensor
    sensor_array.CNTR_IR2 = temp_adc;
    SET_MUX(IR6_MUX);
    ir_state = ADC10;
    break;
    
    case ADC10: // Leftmost center sensor
    sensor_array.CNTR_IR1 = temp_adc;
    SET_MUX(IR7_MUX);
    ir_state = ADC9;
    break;
    
    case ADC9: // second Leftmost sensor
    sensor_array.LEFT_IR2 = temp_adc;
    SET_MUX(IR8_MUX);
    ir_state = ADC8;
    break;
    
    case ADC8: // Leftmost sensor
    sensor_array.LEFT_IR1 = temp_adc;
    SET_MUX(IR1_MUX);
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
//   switch (motor_state){
//     case MTR_OFF:
//       if (PINC & (1<<7)){
//         motor_state = MTR_STRAIGHT;
//       }
//     break;

//     case MTR_STRAIGHT:

//     break;

//     case MTR_RIGHT_TURN:
//     break;

//     case MTR_LEFT_TURN:

//     break;

//     case MTR_SLOW_ZONE:
//     break;

//     default:
//     break;
//   }
// }
