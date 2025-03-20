int main(void){
  // // 1ms periodic interupt
  // TCCR1A |= (1<<COM1A1); // set it to clear output on compare match
  // TCCR1A |= (1<<WGM12) // set to CTC waveform generation mode
  // TCCR1B |= (1<<CS11) // Set prescaler to 8
  // OCR1A = 10000; // 5ms (10000 clocks at 16 MHz (scaled by 8))
  cli();
  // 500ms periodic interupt
  // TCCR1A |= (1<<COM1A1); // set it to clear output on compare match
  // TCCR1A |= (1<<WGM12); // set to CTC waveform generation mode
  // TCCR1B |= (1<<CS11); // Set prescaler to 8
  // OCR1A = 31250; // 5ms (10000 clocks at 16 MHz (scaled by 8)
  // DDRE |= (1<<PE6);

	TCCR1A = 0; // 0b00000000  - TOP is 0xFFFF
	// ICNC1 | ICES1 | - | WGM13 | WGM12 | CS12 | CS11 | CS10
	TCCR1B |= (1<<CS12); //0x0C; // 0b00001XXX	// last 3 bits specify clock source: 0 = no clock source (stopped); 1 = no prescale; 2 = clock / 8; 3 = clock / 64
									// 4 = clock / 256; 5 = clock / 1024; 6 = ext clock on T0 pin falling edge; 7 = ext clock on T0 pin rising edge
	TCCR1C = 0; // not forcing output compare
	TCNT1 = 0; // set timer counter initial value (16 bit value)
	// Interrupts on overflow (every ~1 second)
	//OCR1A = 0xF424;
	OCR1A = 31250; // closer to one second than value above
	//TIMSK1 = 1; // enable timer overflow interrupt	
	TIMSK1 = 2; // enable timer compare match 1A interrupt	
  
  USBCON=0;
  sei();
  
  // set TCCR1A tick
  // TCCR1B
  // TCCR1C
  // TCNT1
  // OCR1A
  // TIMSK1
  while(1){}
  // return 0;
}
// OCR1AH and OCR1AL output compare registers
ISR(TIMER1_COMPA_vect){
  PORTE ^= (1<<PE6);
}
