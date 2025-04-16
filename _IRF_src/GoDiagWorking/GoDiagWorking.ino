#define SET_SERVO_H(x) OCR1A = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
#define SET_SERVO_V(x) OCR1B = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
void setup() {
  // put your setup code here, to run once:
  cli();

  // timer_init();
  SERVO_PWM_init();

  USBCON=0;
  sei();

  set_servos(90,90);
  delay(500);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  go_diagonal();

}


void go_diagonal(void){
  // for (int i = 90; i < 180 ; i++){
  //   delay(20);
  //   set_servos(i,i);
  // }
  delay(20);
  for (int i = ; i >= 0 ; i--){
    delay(20);
    set_servos(i,i);
  }
  delay(50);
  for (int i = 0; i <= 90 ; i++){
    delay(20);
    set_servos(i,i);
  }
  delay(50);
  // set_servos(90,90);
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
