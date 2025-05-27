// PID control reference
// https://www.teachmemicro.com/implementing-pid-for-a-line-follower-robot/

////////////////////////////////////////////////
// Macros
////////////////////////////////////////////////

#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
#define TGLBIT(x,y) x ^= BIT(y)

#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7


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
#define DETECT_TURNING_MARKER !detect_rising_edge(side_sensor_array[0], side_sensor_array[1])
// #define DETECT_TURNING_MARKER side_sensor_array[0]
#define DETECT_OBSTACLE 0 
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
#define IRC1_MUX 
#define IRC2_MUX (1<<MUX0)

#define TURN_LED_ON(port, pin) port |= pin
#define TURN_LED_OFF(port, pin) port &= ~pin
#define TOGGLE_LED(port, pin) port ^= pin
/* Motor pins */ 

// check these values through testing
#define STRAIGHT_SPEED 64
#define SLOW_ZONE_SPEED 32
#define TURN_SPEED 96
#define TURN_SPEED_INCREMENT 2
#define TRACK_ADJUSTMENT_SPEED 1

#define SPEED_LIMIT (64+64)
#define SLOW_ZONE_MTR_SPEED 64
#define OBSTACLE_SPEED 32

#define LEFT_MOTOR OCR0B
#define RIGHT_MOTOR OCR0A 

/* PID Control Coefficients */
// #define PROPORTIONAL_COEFFICIENT 22
// #define INTEGRAL_COEFFICIENT 0.0001
// #define DERIVATIVE_COEFFICIENT 6
#define PROPORTIONAL_COEFFICIENT 20
#define INTEGRAL_COEFFICIENT 0
#define DERIVATIVE_COEFFICIENT 4

// P > D
// D about 1/4 
// start with P go until it becomes unstable (use 30% of result)
// then use D not I 
// use I 

/* Light Setup */
#define STOP_LED_DDR DDRB
#define STRAIGHT_LED_DDR DDRB
#define OBSTACLE_LED_DDR DDRB
#define TURRET_ON_DDR DDRC
#define TURRET_TRACKING_DDR DDRC

#define STOP_LED_PORT PORTB
#define STRAIGHT_LED_PORT PORTB
#define OBSTACLE_LED_PORT PORTB
#define TURRET_ON_PORT PORTC
#define TURRET_TRACKING_PORT PORTC

#define STOP_LED_PIN PIN3
#define STRAIGHT_LED_PIN PIN2
#define OBSTACLE_LED_PIN PIN1
#define TURRET_ON_PIN PIN6
#define TURRET_TRACKING_PIN PIN7

////////////////////////////////////////////////
// Type Definitions
////////////////////////////////////////////////

typedef enum {
  IDLE,
  STRAIGHT,
  SRT_TO_TRN_STATE,
  TRN_TO_SRT_STATE,
  TURNING,
  SLOW_ZONE_STRAIGHT,
  SLOW_ZONE_TURNING,
  ENTER_INIT_SLOW_ZONE,
  EXIT_INIT_SLOW_ZONE,
  ENTER_SLOW_ZONE,
  EXIT_SLOW_ZONE,
  INITIAL_SLOW_ZONE,
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
  HOLDING_IR,
} LINE_IR_TYPE;

typedef enum {
  ADC0,   // S1
  ADC1,   // S2
  HOLDING_CS,
} COLOUR_IR_TYPE;

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

typedef enum{
  LINE_SENSING,
  COLOUR_SENSING,
} ADC_STATE_TYPE;
////////////////////////////////////////////////
// Function Prototyping
////////////////////////////////////////////////

void timer_init(void);
void ADC_init(void);
void MOTOR_PWM_init(void);
void robot_state_machine(void);
void motor_state_machine(void);
void line_ir_state_machine(void);
void PID_calc(void);
void PID_init(void);
void turn_calc(void);
void set_motors(int L , int R);
void COMMS_init(void);
void LED_init(void);
void CLEAR_LEDS(void);
void SET1_LED(int PORT, int PIN);
void SET2_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2);
void SET3_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2, int PORT_3, int PIN_3);
void SET4_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2, int PORT_3, int PIN_3, int PORT_4, int PIN_4);
int detect_rising_edge(int current_sensor1, int current_sensor2);

////////////////////////////////////////////////
// Global Variable Declaration
////////////////////////////////////////////////

volatile BOT_STATE_TYPE bot_state = STRAIGHT; 
volatile int side_sensor_array[2] = {0,0};
volatile int line_sensor_array[4] = {0,0,0,0};
volatile int colour_sensor_array[3] = {0,0,0};
volatile MOTOR_STATE_TYPE motor_state = MTR_OFF;
volatile LINE_IR_TYPE line_ir_state = ADC4;
volatile COLOUR_IR_TYPE colour_ir_state = HOLDING_CS;

volatile int temp_adc = 0;
volatile ADC_STATE_TYPE adc_state = LINE_SENSING; 

// PID Variables
volatile float P, I, D, LP;
const float Kp = PROPORTIONAL_COEFFICIENT;
const float Ki = INTEGRAL_COEFFICIENT;
const float Kd = DERIVATIVE_COEFFICIENT;
volatile int maxspeed = 128;
volatile int basespeed = 128;
volatile int r_mtr_speed, l_mtr_speed;
volatile float error, correction, sp;
volatile int position, sensor_average, sensor_sum;
volatile int currentspeedR = 64;
volatile int currentspeedL = 64;
volatile bool slow_zone = false;
volatile int prev_sensor1 = 0;
volatile int prev_sensor2 = 0;
// volatile int current_corner_sensor = 0;
volatile bool signal_from_turret = false;
volatile int slow_zone_count = 0;
volatile bool slow_zone_check = false;

////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

void setup() {
  // Pause interupts before initialisation
  cli();
  // Initialisation
  ADC_init(); 
  MOTOR_PWM_init();
  // COMMS_init();
  LED_init();
  // Reset interupts after initialisation
  sei();
  Serial1.begin(9600);
  Serial.begin(9600);
}

void loop(){
  PID_calc();
  turn_calc();

  // robot_state_machine();

  // Serial.println("--------------------");
  // Serial.println(line_sensor_array[0]);
  // Serial.println(line_sensor_array[1]);
  // Serial.println(line_sensor_array[2]);
  // Serial.println(line_sensor_array[3]);
  // Serial.println(line_sensor_array[4]);
  // Serial.println(line_sensor_array[5]);
  // Serial.println("--------------------");
  // Serial.println("Logic Control");
  // Serial.println("---------------");
  // Serial.print("Average ==> ");
  // Serial.println(sensor_sum);
  // Serial.print("Sum ==> ");
  // Serial.println(sensor_average);
  // Serial.print("Position ==> ");
  // Serial.println(position);
  // Serial.print("Error ==> ");
  // Serial.println(error);
  // Serial.print("Current R Speed ==> ");
  // Serial.println(currentspeedR);
  // Serial.print("Current L Speed ==> ");
  // Serial.println(currentspeedL);
  // Serial.println("---------------");

  // delay(100);
  
}

////////////////////////////////////////////////
// Control System and State Machines
////////////////////////////////////////////////

void robot_state_machine(void){

  switch(bot_state){
    case IDLE:
      SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
      // max_speed = SLOW_ZONE_SPEED;
      SETBIT(PORTB, PIN1);
      maxspeed = SLOW_ZONE_SPEED;
      bot_state = INITIAL_SLOW_ZONE;
    break;

    case INITIAL_SLOW_ZONE:
      PID_calc();
      turn_calc();
      if(colour_sensor_array[0]){
        bot_state = EXIT_INIT_SLOW_ZONE;
        maxspeed = STRAIGHT_SPEED;
      }
    break;
    
    case STRAIGHT:
      PID_calc();
      turn_calc();
      // if(DETECT_TURNING_MARKER){
      if(side_sensor_array[0]){  
        CLEAR_LEDS();
        bot_state = SRT_TO_TRN_STATE;
      }
      if(colour_sensor_array[0]){
        Serial1.println('1');
        SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN);
        bot_state = ENTER_SLOW_ZONE;
        maxspeed = SLOW_ZONE_SPEED;
      } 
    break;

    case SRT_TO_TRN_STATE:
      PID_calc();
      turn_calc();
      if (!side_sensor_array[0]){
        if (!slow_zone_check){
          bot_state = TURNING;  
        } else {
          bot_state = SLOW_ZONE_TURNING;
        }
      } 
    break;

    case TRN_TO_SRT_STATE:
      PID_calc();
      turn_calc();
      if (!side_sensor_array[0]){
        if (!slow_zone_check){
          bot_state = STRAIGHT;  
        } else {
          bot_state = SLOW_ZONE_STRAIGHT;
        }
      } 
    break;

    case TURNING:
      PID_calc();
      turn_calc();
      // if(DETECT_TURNING_MARKER){
      if(side_sensor_array[0]){
        // SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
        SETBIT(PORTB, PIN1);
        bot_state = TRN_TO_SRT_STATE;
      }
      if(ENTER_SLOW_ZONE){
        Serial1.println('1');
        SET1_LED(TURRET_ON_PORT, TURRET_ON_PIN);
        bot_state = SLOW_ZONE_TURNING;
        maxspeed = SLOW_ZONE_SPEED;
      } 
    break;

    case SLOW_ZONE_STRAIGHT:
      PID_calc();
      turn_calc();

      signal_from_turret = check_input_signal();

      if(side_sensor_array[0]){
        if(signal_from_turret){
          SET2_LED(TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else{
          SET1_LED(TURRET_ON_PORT, TURRET_ON_PIN);
        }
        bot_state = SLOW_ZONE_TURNING;
      } else if(DETECT_GREEN(0)){
        Serial1.println('0');
        bot_state = STRAIGHT;
        SET1_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN);
        set_motors(SPEED_LIMIT, SPEED_LIMIT);
      } 
    
      if(DETECT_OBSTACLE){
        set_motors(OBSTACLE_SPEED, OBSTACLE_SPEED);
        SET4_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN, OBSTACLE_LED_PORT, OBSTACLE_LED_PIN);
      } else {
        if(signal_from_turret){
          SET3_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else {
          SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        }
      }
    break;

    case SLOW_ZONE_TURNING:
      PID_calc();
      turn_calc();

      signal_from_turret = check_input_signal();

      if(DETECT_TURNING_MARKER){
        if(signal_from_turret){
          SET3_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN, TURRET_TRACKING_PORT, TURRET_TRACKING_PIN);
        } else{
          SET2_LED(STRAIGHT_LED_PORT, STRAIGHT_LED_PIN, TURRET_ON_PORT, TURRET_ON_PIN);
        }
        bot_state = SLOW_ZONE_STRAIGHT;
      } else if(DETECT_GREEN(0)){
        Serial1.println('0');
        bot_state = TURNING;
        CLEAR_LEDS();
        set_motors(SPEED_LIMIT, SPEED_LIMIT);
      } 
    
    break;  
    
    case EXIT_INIT_SLOW_ZONE:
      PID_calc();
      turn_calc();
      if (!colour_sensor_array[0]){
        slow_zone_count = 1;
        bot_state = STRAIGHT;
      } 
    break;
    
    case ENTER_INIT_SLOW_ZONE:
      PID_calc();
      turn_calc();
      if (!colour_sensor_array[0]){
        slow_zone_count = 4;
        bot_state = INITIAL_SLOW_ZONE;
      }
    break;

    case ENTER_SLOW_ZONE:
      PID_calc();
      turn_calc();
      if (!colour_sensor_array[0]){
        slow_zone_count = 2;
        bot_state = SLOW_ZONE_STRAIGHT;
      }
    break;
    
    case EXIT_SLOW_ZONE:
      PID_calc();
      turn_calc();
      if (!colour_sensor_array[0]){
        slow_zone_count = 3;
        bot_state = STRAIGHT;
      }
    break;
  }
}
// todo: make the pwm init run when a button is pressed
void sensor_value_average(void){
  sensor_average = 0;
  sensor_sum = 0;
  // todo: (?) make a for loop?
  sensor_average = (line_sensor_array[0] * -2 * 1000) + (line_sensor_array[1] * -1 * 1000) + (line_sensor_array[2] * 1 * 1000) + (line_sensor_array[3] * 2 * 1000);
  // sensor_average = (line_sensor_array[0] * -2) + (line_sensor_array[1] * -1) + (line_sensor_array[2] * 1) + (line_sensor_array[3] * 2);
  sensor_sum = line_sensor_array[0] + line_sensor_array[1] + line_sensor_array[2] + line_sensor_array[3];
  if (sensor_average == 0){
    position = 0;
  } else if ((sensor_average == 0) & (side_sensor_array[0] == 1) ){
    position = 0;
  } else {
    position = sensor_average / sensor_sum;
  }
  
}

void PID_calc(void){
  sensor_value_average();
  // error = position - sp;
  // if(!(line_sensor_array[0] & line_sensor_array[5])){
  // if ((error > -1) & (error < 1)){
  //   error = 0;
  // }
  error = position ;
  P = error;
  I += P;
  D = P - LP;
  LP = P;
  correction = int(Kp*P + Ki*I + Kd*D);
  // } else {
  //   correction = 0;
  // }
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
void MOTOR_PWM_init(void){
  // Set pins b7 and d0 to output to expose pwm signal to pin
  SETBIT(DDRB, PB7);
  SETBIT(DDRD, PD0);
  
  // todo: Set this up to be more readable ie why we set the bits how we do
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);
  TCCR0B |= (1<<CS00);

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
  // SETBIT(DDRB, PIN1);
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
void SET2_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2){
  CLEAR_LEDS();
  SETBIT(PORT_1, PIN_1);
  SETBIT(PORT_2, PIN_2);
}
void SET3_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2, int PORT_3, int PIN_3){
  CLEAR_LEDS();
  SETBIT(PORT_1, PIN_1);
  SETBIT(PORT_2, PIN_2);
  SETBIT(PORT_3, PIN_3);
}
void SET4_LED(int PORT_1, int PIN_1, int PORT_2, int PIN_2, int PORT_3, int PIN_3, int PORT_4, int PIN_4){
  CLEAR_LEDS();
  SETBIT(PORT_1, PIN_1);
  SETBIT(PORT_2, PIN_2);
  SETBIT(PORT_3, PIN_3);
  SETBIT(PORT_4, PIN_4);
}
int detect_rising_edge(int current_sensor1, int current_sensor2){
  int val_changed = 0; int rising_edge = 0;
  val_changed = (!prev_sensor1 ^ !current_sensor1);
  rising_edge = (val_changed & !current_sensor1);
  return rising_edge;
} 
bool check_input_signal(void){
    if (Serial1.available() >= 0) {
      char receivedData = Serial1.read();   // read one byte from serial buffer and save to receivedData
      if (receivedData == '1') {
        return true;
      }
      else if (receivedData == '0') {
        return false;
      }
  }
}
int check_slow_zone_state(void){
  if (slow_zone_count == 1){
    maxspeed = STRAIGHT_SPEED;
  } else if (slow_zone_count == 2){
    
  }
}
////////////////////////////////////////////////
// Interupt Service Routines
////////////////////////////////////////////////

// ADC conversion interupt
ISR(ADC_vect){
  temp_adc = ADCH;
  
  switch(adc_state){
    case LINE_SENSING:
      if (temp_adc < WHITE_LINE_THRESHOLD) {
        temp_adc = 1;
      } else{
        temp_adc = 0;
      }

      switch (line_ir_state){ 
        case ADC4: // Rightmost sensor
        line_sensor_array[0] = temp_adc;
        SET_MUX(IR2_MUX);
        ADCSRB &= ~(1<<MUX5);
        line_ir_state = ADC5;
        break;
        
        case ADC5: // Second Rightmost sensor
        line_sensor_array[1] = temp_adc;
        SET_MUX(IR3_MUX);
        line_ir_state = ADC6;
        break;
        
        case ADC6: // Rightmost center sensor
        prev_sensor1 = side_sensor_array[0];
        side_sensor_array[0] = temp_adc;
        // line_sensor_array[4] = temp_adc;
        SET_MUX(IR4_MUX);
        line_ir_state = ADC7;
        break;
        
        case ADC7: // Second Rightmost center sensor
        side_sensor_array[1] = temp_adc;
        SET_MUX(IR5_MUX);
        // line_sensor_array[3] = temp_adc;
        ADCSRB |= (1<<MUX5);
        line_ir_state = ADC11;
        break;
        
        case ADC11: // Second Leftmost center sensor
        line_sensor_array[2] = temp_adc;
        SET_MUX(IR6_MUX);
        ADCSRB |= (1<<MUX5);
        line_ir_state = ADC10;
        break;
        
        case ADC10: // Leftmost center sensor
        // line_sensor_array[3] = temp_adc;
        SET_MUX(IR7_MUX);
        ADCSRB |= (1<<MUX5);
        line_ir_state = ADC9;
        break;
        
        case ADC9: // second Leftmost sensor
        line_sensor_array[1] = temp_adc;
        CLEAR_MUX;
        ADCSRB |= (1<<MUX5);
        line_ir_state = ADC8;
        break;
        
        case ADC8: // Leftmost sensor
        // line_sensor_array[4] = temp_adc;
        CLEAR_MUX;
        ADCSRB &= ~(1<<MUX5);
        line_ir_state = ADC4;
        colour_ir_state = ADC0;
        adc_state = COLOUR_SENSING;
        break;

        case HOLDING_IR:

        break;
        
        default:
          SET_MUX(IR1_MUX);
        break;
      }
    break;

    case COLOUR_SENSING:
      switch(colour_ir_state){
        case ADC0:
          if (temp_adc < RED_LIGHT_LEVEL) {
            temp_adc = 1;
          } else{
            temp_adc = 0;
          }
          colour_sensor_array[0] = temp_adc;
          CLEAR_MUX;
          ADCSRB &= ~(1<<MUX5);
          SET_MUX(IRC2_MUX); // change
          colour_ir_state = ADC1;

        break;

        case ADC1:
          if (temp_adc < GREEN_LIGHT_LEVEL) {
            temp_adc = 1;
          } else{
            temp_adc = 0;
          }        
          colour_sensor_array[1] = temp_adc;
          CLEAR_MUX;
          SET_MUX(IR1_MUX); // change
          line_ir_state = ADC4;
          colour_ir_state = ADC1;
          adc_state = LINE_SENSING;
        break;

        case HOLDING_CS:
        break;
      }
    break;
  }
  
  
  // start conversion again
  ADCSRA |= (1<<ADSC);
}

