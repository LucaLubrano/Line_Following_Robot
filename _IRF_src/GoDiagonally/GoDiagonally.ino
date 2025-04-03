#define SET_SERVO_H(x) OCR1A = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
#define SET_SERVO_V(x) OCR1B = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
////////////////////////////////////////////////
// Main
////////////////////////////////////////////////

int main(void){
  cli();

  // timer_init();
  SERVO_PWM_init();

  USBCON=0;
  sei();

  set_servos(90,90);

  while(1){
    go_diagonal();
  }
}

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

void go_diagonal(void){
  for (int i = 0; i < 181 ; i++){
    set_servos(i,i);
  }
  for (int i = 180; i >= 0 ; i--){
    set_servos(i,i);
  }
  set_servos(90,90);
}

void set_servos(int H , int V){
  SET_SERVO_H(H);
  SET_SERVO_V(V);
}

void SERVO_PWM_init(void){
  DDRB |= (1<<5);
  DDRB |= (1<<6);
  DDRD |= 1;
  
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (0<<WGM10);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | 0b100;
  // TCCR1A = 0b10100011;
  // TCCR1B = 1;
  ICR1 = 1249;
  OCR1A = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
  OCR1B = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
}

ISR(TIMER1_COMPA_vect){
  go_diagonal();
}