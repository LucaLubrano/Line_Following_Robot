#define IR1_MUX (1<<MUX2) // right most
#define IR2_MUX ((1<<MUX2) | (1<<MUX0))// second right most
#define IR3_MUX 0b000110 // right most center
#define IR4_MUX 0b000111 // middle right center
#define IR5_MUX 0b100011 // middle left center
#define IR6_MUX 0b100010 // left most center
#define IR7_MUX 0b100001 // second left most
#define IR8_MUX 0b100000 // left most
#define CLEAR_ADMUX ADMUX &= 0b11100000;
#define ADMUX_CONFIG (1<<REFS0) | (1<<ADLAR)

#define IR11 ADMUX_CONFIG | IR1_MUX
#define IR22 ADMUX_CONFIG | IR2_MUX

volatile int adc_result0;
volatile int adc_result1;
volatile int adc_result2;
volatile int adc_result3;

// interupt when conversion finishes
ISR(ADC_vect){
  ////////////////////////////////
  // This code works
  int adcval = ADCH;
  //read conversion result
  if (ADMUX & (1<<MUX0)){
    adc_result0 = adcval;
    // CLEAR_ADMUX;
  } else {
    adc_result1 = adcval;
    // CLEAR_ADMUX;
    // ADMUX |= (IR2_MUX);
  } 
    // ADMUX |= IR1_MUX;
  ADMUX ^= (1<<MUX0);
  ////////////////////////////////
  



  // if (adc_result3){
  //   ADMUX = ADMUX_CONFIG | (1<<MUX2);
  // } else {
  //   ADMUX = ADMUX_CONFIG | (1<<MUX2) | (1<<MUX0);
  // }
  //   ADMUX = ((1<<MUX2) | (1<<MUX0));
  // } else if (ADMUX & ((1<<MUX2) | (1<<MUX0))){
  //   adc_result1 = ADCH;
  //   PORTB |= (1<<0);
  //   ADMUX = ((1<<MUX2) | (1<<MUX1));
  // } else if (ADMUX & ((1<<MUX2) | (1<<MUX1))){
  //   adc_result2 = ADCH;
  //   ADMUX = ((1<<MUX2) | (1<<MUX1) | (1<<MUX0));
  // } else if (ADMUX & ((1<<MUX2) | (1<<MUX1) | (1<<MUX0))){
  //   adc_result3 = ADCH;
  //   ADMUX = (1<<MUX2);
  // } 
  // if (ADMUX & 0b000100){
  //   adc_result0 = ADCH;
  // }
  // } else if (ADMUX & IR2_MUX){
  //   adc_result0 = ADCH;  
  // } else if (ADMUX & IR3_MUX){
  //   adc_result0 = ADCH;  
  // } else if (ADMUX & IR4_MUX){
  //   adc_result0 = ADCH;   
  // } else if (ADMUX & IR5_MUX){
  //   adc_result0 = ADCH;   
  // } else if (ADMUX & IR6_MUX){
  //   adc_result0 = ADCH;   
  // } else if (ADMUX & IR7_MUX){
  //   adc_result0 = ADCH;  
  // } else if (ADMUX & IR8_MUX){
  //   adc_result0 = ADCH;  
  // } 

  // Start conversion again
  // ADMUX ^= (1<<MUX2);
  


    ADCSRA |= (1<<ADSC);

}

int main(void){
  
  DDRE |= (1<<6);
  DDRB |= (1<<0);
  DDRB |= (1<<1);
  DDRB |= (1<<2);

  // PORTE |= (1<<6);
  


  // Timer 0 Setup (PWM SET UP)
  TCCR0A |= (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<< WGM00);
  TCCR0B |= (1<<CS02);
  DDRB |= (1<<PB7);
  DDRD |= (1<<PD0);

  // Set up ADC
  ADMUX |= (1<<REFS0) | (1<<ADLAR);
  ADCSRA |= (1<<ADEN) | (1<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
  ADCSRB |= (1<<ADHSM);
  
  // Start ADC
  ADCSRA |= (1<<ADSC);

  USBCON=0;
  sei();

  ADMUX |= (1<<MUX2);
  // int divd = 256/4;
  while(1){
    // int divd = 256/4;
    // if (adc_result0 < divd){
    //   PORTE |= (1<<6);
    //   PORTB &= ~(1<<0);
    //   PORTB &= ~(1<<1);
    //   PORTB &= ~(1<<2);
    // } else if ((adc_result0 > divd) && ( adc_result0 < (2*divd))){
    //   PORTE |= (1<<6);
    //   PORTB |= (1<<0);
    //   PORTB &= ~(1<<1);
    //   PORTB &= ~(1<<2);
    // } else if ((adc_result0 > (2*divd)) && ( adc_result0 < (3*divd))){
    //   PORTE |= (1<<6);
    //   PORTB |= (1<<0);
    //   PORTB |= (1<<1);
    //   PORTB &= ~(1<<2);
    // } else if ((adc_result0 > (3*divd)) && ( adc_result0 < (4*divd))){
    //   PORTB |= (1<<2);
    //   PORTE |= (1<<6);
    //   PORTB |= (1<<0);
    //   PORTB |= (1<<1);
    // }

    int white = 220;
    int black = 180;

    if (adc_result0 < white){
      PORTE |= (1<<6);
    } else if (adc_result0 > black){
      PORTE &= ~(1<<6);
    }

    if (adc_result1 < white){
      PORTB |= (1<<0);
    } else if (adc_result1 > black){
      PORTB &= ~(1<<0);
    }

    if (adc_result2 > white){
      PORTB |= (1<<1);
    } else if (adc_result1 < black){
      PORTB &= ~(1<<1);
    }

    if (adc_result3 > white){
      PORTB |= (1<<2);
    } else if (adc_result1 < black){
      PORTB &= ~(1<<2);
    }

    OCR0A = adc_result0;
    // OCR0B = -adc_result0;
    // else {
    //   PORTE &= ~(1<<6);
    // }

  }

}

