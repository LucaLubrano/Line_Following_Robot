// int main(void){
  
//   // LED set 
//   DDRE |= (1<<PE6);
#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
//   // Set up ADC
//   ADMUX |= (1<<REFS0) | (1<<ADLAR);
//   ADCSRA |= (1<<ADEN) | (1<< ADATE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
//   ADCSRB |= (1<<ADHSM);
  
//   // Start ADC
//   ADCSRA |= (1<<ADSC);

//   while(1){
//     if (ADCH > 128){
//       PORTE |= (1<<PE6);
//     } else {
//       PORTE &= ~(1<<PE6);
//     }
//   }

// }
#define ADMUX_PRESET (1<<REFS0) | (1<<ADLAR)
#define S1 (1<<MUX2)
#define S2 (1<<MUX2) | (1<<MUX0)
#define S3 (1<<MUX2) | (1<<MUX1)
#define S4 (1<<MUX2) | (1<<MUX1) | (1<<MUX0)
#define WHITE_LINE_THRESHOLD 128
#define BLACK_LINE_THRESHOLD 128
#define LED_0 6
#define LED_1 0
#define LED_2 1
#define LED_3 2
/*
S1 = PF4 = ADC4 = MUX ( 0b000100) 
S2 = PF5 = ADC5 = MUX ( 0b000101) 
S3 = PF6 = ADC6 = MUX ( 0b000110) 
S4 = PF7 = ADC7 = MUX ( 0b000111) 
S5 = PB4 = ADC11 = MUX ( 0b100011)
S6 = PD7 = ADC10 = MUX ( 0b100010)
S7 = PD6 = ADC9 = MUX ( 0b100001) 
S8 = PD4 = ADC8 = MUX ( 0b100000) 
*/

static int adc0;
static int adc1;
static int adc2;
static int adc3;
volatile int temp_result;
typedef enum {
  ADC0,
  ADC1,
  ADC2,
  ADC3,
} IR_SENSE_STATE;

volatile IR_SENSE_STATE ir_state = ADC0;

int main(void){
  
  // Timer 0 Setup (PWM SET UP)
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<< WGM00);
  TCCR0B |= (1<<CS02);
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);
  
  

  // Led set up
  DDRE |= (1<<6);
  DDRB |= (1<<0);
  DDRB |= (1<<1);
  DDRB |= (1<<2);

  // Set up ADC
  ADMUX |= (1<<REFS0) | (1<<ADLAR);
  ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB |= (1<<ADHSM);

  ADMUX |= (1<<MUX0);
  

  USBCON=0;
  sei();

  // Start ADC
  ADCSRA |= (1<<ADSC);


  while(1){
    if(adc3 < BLACK_LINE_THRESHOLD){
      PORTE &= ~(1<<LED_0);
    } else if (adc3 > WHITE_LINE_THRESHOLD) {
      PORTE |= (1<<LED_0);
    }

    if (adc2 < BLACK_LINE_THRESHOLD){
      PORTB &= ~(1<<LED_1);
    } else if (adc2 > WHITE_LINE_THRESHOLD) {
      PORTB |= (1<<LED_1);
    }

    if (adc1 < BLACK_LINE_THRESHOLD){
      PORTB &= ~(1<<LED_2);
    } else if (adc1 > WHITE_LINE_THRESHOLD) {
      PORTB |= (1<<LED_2);
    }

    if (adc0 < BLACK_LINE_THRESHOLD){
      PORTB &= ~(1<<LED_3);
    } else if (adc0 > WHITE_LINE_THRESHOLD) {
      PORTB |= (1<<LED_3);
    }
    OCR0A = adc1;
    OCR0B = adc2;

  }

}

// interupt when conversion finishes
ISR(ADC_vect){
/////////////////////////////////////////  
  // // read conversion result
  // if (ADMUX & (1<<MUX0)){
  //   adc0 = ADCH;  
  // } else {
  //   adc1 = ADCH;
  // }
  
  // // adc0 = ADCH;  

  // // switch 
  // ADMUX ^= (1<<MUX0);

  // // Start conversion again
  // ADCSRA |= (1<<ADSC);
/////////////////////////////////////////


  temp_result = ADCH;
  switch (ir_state){

    case ADC0:
    adc0 = temp_result;
    ADMUX &= 0b11100000;
    ADMUX |= S2;
    ir_state = ADC1;
    // ADCSRA |= (1<<ADSC);
    break;

    case ADC1:
    adc1 = temp_result;
    ADMUX &= 0b11100000;
    ADMUX |= S3;
    ir_state = ADC2;
    // ADCSRA |= (1<<ADSC);
    break;

    case ADC2:
    adc2 = temp_result;
    ADMUX &= 0b11100000;
    ADMUX |= S4;
    ir_state = ADC3;
    // ADCSRA |= (1<<ADSC);
    break;

    case ADC3:
    adc3 = temp_result;
    ADMUX &= 0b11100000;
    ADMUX |= S1;
    ir_state = ADC0;
    // ADCSRA |= (1<<ADSC);
    break;

    default:
    break;

  }
  
  // Start conversion again
  ADCSRA |= (1<<ADSC);

}

