// put your setup code here, to run once:
#define IR_SENSING_THRESHOLD 180

#define CLEAR_MUX ADMUX &= 0b11100000
#define SET_MUX(ADC_CHANNEL) CLEAR_MUX; ADMUX |= ADC_CHANNEL

#define IR1_MUX (1<<MUX2)                           // ADC4   // right most
#define IR2_MUX (1<<MUX2) | (1<<MUX0)               // ADC5   // second right most
#define IR3_MUX (1<<MUX2) | (1<<MUX1)               // ADC6   // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // ADC7   // middle right center

#define IR1_MUX (1<<MUX2)                           // ADC4   // right most
#define IR2_MUX (1<<MUX2) | (1<<MUX0)               // ADC5   // second right most
#define IR3_MUX (1<<MUX2) | (1<<MUX1)               // ADC6   // right most center
#define IR4_MUX (1<<MUX2) | (1<<MUX1) | (1<<MUX0)   // ADC7   // middle right center
typedef enum {
  ADC4,   // S1
  ADC5,   // S2
  ADC6,   // S3
  ADC7,   // S4
} IR_STATE_TYPE;

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

void setup() {
  cli();
  ADC_init();
  USBCON=0;
  sei();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("-----");
  Serial.println(sensor_array[0]);
  Serial.println(sensor_array[1]);
  Serial.println(sensor_array[2]);
  Serial.println(sensor_array[3]);
  delay(20);
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
  // if (temp_adc > IR_SENSING_THRESHOLD) {
  // if (temp_adc > IR_SENSING_THRESHOLD) {
  //   temp_adc = 1;
  // } else{
  //   temp_adc = 0;
  // }

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
    // line_sensor_array[3] = temp_adc;
    SET_MUX(IR4_MUX);
    ir_state = ADC7;
    break;
    
    case ADC7: // Second Rightmost center sensor
    sensor_array[3] = temp_adc;
    // line_sensor_array[2] = temp_adc;
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