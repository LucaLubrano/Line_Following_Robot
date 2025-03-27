void setup() {
  // put your setup code here, to run once:
  DDRC &= ~(1<<7);
  PORTB &= ~(1<<7);
  DDRB |= (1<<2);
  PORTB |= (1<<2);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (PINC & (1<<7)){
    PORTB ^= (1<<2);
  }
}
