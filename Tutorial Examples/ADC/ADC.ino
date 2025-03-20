// int main(void){
  
//   // LED set 
//   DDRE |= (1<<PE6);

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

volatile int adc_result0;
volatile int adc_result1;

// interupt when conversion finishes
ISR(ADC_vect){
  
  // read conversion result
  if (ADMUX & (1<<MUX0)){
    adc_result0 = ADCH;  
  } else {
    adc_result1 = ADCH;
  }
  
  // adc_result0 = ADCH;  

  // switch 
  ADMUX ^= (1<<MUX0);

  // Start conversion again
  ADCSRA |= (1<<ADSC);


}

int main(void){
  
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

  while(1){

    // pwm output set
    OCR0A = adc_result0;
    OCR0B = adc_result1;

  }

}

