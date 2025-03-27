#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
#define TGLBIT(x,y) x ^= BIT(y)
void setup() {
  // Set pins b7 and d0 to output to expose pwm signal to pin
  DDRD |= (1<<0);
  DDRB |= (1<<7);
  // SETBIT(DDRB, PB7);
  // SETBIT(DDRD, PD0);

  // put your setup code here, to run once:
  TCCR0A = (1<< COM0A1) | (1<<COM0B1) | (1<<WGM01) | (1<< WGM00);
  TCCR0B = (1<<CS00);

  // // set pits b0 and e6 to outputs for phase control
  // SETBIT(DDRB, PB0);
  // SETBIT(DDRE, PE6);

  // // // set output direction values
  // SETBIT(PORTB, PB0);
  // SETBIT(PORTE, PE6);

  OCR0B = -128;
  OCR0A = -128;

}

void loop() {
  // put your main code here, to run repeatedly:
  // OCR0A += 1;
  // OCR0B += 1;

  // if ((OCR0A >= 255) || (OCR0B >= 255)){
  //   OCR0A = 0;
  //   OCR0B = 0;
  // }
}
