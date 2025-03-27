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


// int main(void)
// {
// // DDRD= 0xFF ;
// // DDRA=0x00;
// // PORTA =0x00;
// DDRB |= (1<<PB5);
// // // FAST PWM
// // TCCR1A = 1<<COM1A0 | 1<<COM1A1 | 1<<WGM11 |1<<COM1B0|1<<COM1B1 ;
// // TCCR1B = 1<<WGM12|1<<WGM13 ;
// // ICR1 = 1249 ;

// // TCCR1B |= 1<<CS12 ;
// // OCR1A = 1300;

// // PORTB |= (1 << PB5);    // Set PB5 as output

// TCCR1A |= (1 << COM1A1);                // Configure PWM pin
// TCCR1B |= (1 << WGM13) | (0b100);   // Start timer with mode 8


// ICR1 = 1249;	// Adjust frequency
// OCR1A = 1/20 * 1249;	// Adjust duty cycle
// // TCCR1A |= (1 << COM1A1);                // Configure PWM pin
// // TCCR1B |= (1 << WGM13) | (0b100);   // Start timer with mode 8
// // TCCR1A = (1<<COM1A1) | (1<<WGM11) ;
// // TCCR1B = (1<<WGM13) | (1<<WGM12) | (0b100);
// // ICR1 = 1249;

// // ICR1 = 1249;	// Adjust frequency
// // OCR1A = 1/20 * 1249;	// Adjust duty cycle

// while (1)
// {
//   // delay(100);
//   // OCR1A += 1/20 * 1249;
//   // OCR1A += int(1/20 * 1249);
//   //  if(PORTA==0x08) //if pina3 is 1
//   //  {
//   //   OCR1A = ICR1 -2000 ;   //turn servo 1
//   //   _delay_ms(1000);
    
//   //   OCR1B=ICR1-2000;    //turn servo 2
//   //   _delay_ms(1000);
//   //   }
//   //   if(PORTA==0x00)// if pina3 is 0
//   //   {
//   //     OCR1A = ICR1 -900 ; //turn servo 1 in the other direction
//   //     _delay_ms(1000); 
      
//   //     OCR1B=ICR1-900;     //turn servo 2 in the other direction
//   //     _delay_ms(1000);

//   //   }
// }

    
// return 0 ;

// }