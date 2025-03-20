#define SET_BIT(reg, set_bit) reg |= set_bit
#define IR1_MUX 0b000100 // right most
#define IR2_MUX 0b000101 // second right most
#define IR3_MUX 0b000110 // right most center
#define IR4_MUX 0b000111 // middle right center
#define IR5_MUX 0b100011 // middle left center
#define IR6_MUX 0b100010 // left most center
#define IR7_MUX 0b100001 // second left most
#define IR8_MUX 0b100000 // left most
#define LED_0 PIN6
#define LED_1 PIN0
#define LED_2 PIN1
#define LED_3 PIN2
#define LED_4 PIN7
#define LED_5 PIN0
#define LED_6 PIN6
#define LED_7 PIN5

#define WHITE_LINE_THRESHOLD 220
#define BLACK_LINE_THRESHOLD 180

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

// volatile int sens1, sens2, sens3, sens4;
volatile IR_DATA sensor_array = {0,0,0,0,0,0,0,0};

int main(void){

  TCCR1A = 0;
  TCCR1B |= (1<<CS12);
  TCCR1C = 0;
  TCNT1 = 0;
  OCR1A = 31250;
  TIMSK1 = 2;

    // Set up ADC
  ADMUX |= (1<<REFS0) | (1<<ADLAR);
  ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB |= (1<<ADHSM);

  // Start ADC
  ADCSRA |= (1<<ADSC);

  ADMUX |= (1<<IR1_MUX);

  // Set all to outputs
  DDRE |= (1<<6);
  DDRB |= (1<<0);
  DDRB |= (1<<1);
  DDRB |= (1<<2);

  USBCON=0;
  sei();

  while(1){
    PORTE |= LED_0;
  }
}

ISR(TIMER1_COMPA_vect){
  if (sensor_array.CNTR_IR4 < BLACK_LINE_THRESHOLD){
    PORTE |= LED_0;
  } else if (sensor_array.CNTR_IR3 > WHITE_LINE_THRESHOLD){
    PORTB &= ~LED_1;
  } else if (sensor_array.CNTR_IR2 > WHITE_LINE_THRESHOLD){
    PORTB &= ~LED_2;
  } else if (sensor_array.CNTR_IR1 < BLACK_LINE_THRESHOLD){
    PORTB |= LED_3;
  }
}



ISR(ADC_vect){
  // set variable based on conversion being completed
  // todo: make this a state machine
  if (ADMUX & (1<<IR1_MUX)){
    sensor_array.RIGHT_IR2 = ADCH;
    ADMUX |= IR2_MUX;
  } else if (ADMUX & (1<<IR2_MUX)){
    sensor_array.RIGHT_IR1 = ADCH;  
    ADMUX |= IR3_MUX;
  } else if (ADMUX & (1<<IR3_MUX)){
    sensor_array.CNTR_IR4 = ADCH;  
    ADMUX |= IR4_MUX;
  } else if (ADMUX & (1<<IR4_MUX)){
    sensor_array.CNTR_IR3 = ADCH;   
    ADMUX |= IR5_MUX;
  } else if (ADMUX & (1<<IR5_MUX)){
    sensor_array.CNTR_IR2 = ADCH;   
    ADMUX |= IR6_MUX;
  } else if (ADMUX & (1<<IR6_MUX)){
    sensor_array.CNTR_IR1 = ADCH;   
    ADMUX |= IR7_MUX;
  } else if (ADMUX & (1<<IR7_MUX)){
    sensor_array.LEFT_IR2 = ADCH;  
    ADMUX |= IR8_MUX;
  } else if (ADMUX & (1<<IR8_MUX)){
    sensor_array.LEFT_IR1 = ADCH;  
    ADMUX |= IR1_MUX;
  }
  // start conversion again
  ADCSRA |= (1<<ADSC);
}