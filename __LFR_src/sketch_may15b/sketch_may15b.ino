#define SET_SERVO_H(x) OCR1A = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1
#define SET_SERVO_V(x) OCR1B = ((0.5/20.0) + ((x/180.0) * (2))/20.0) * ICR1

volatile int current_angle_H = 90;
volatile int current_angle_V = 90;

void setup() {
  // put your setup code here, to run once:  
  SERVO_PWM_init();

  OCR1A = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
  OCR1B = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(200);
  for (int i = 0; i < 90; i++){
    inc_servo_H();
    inc_servo_V();
    delay(30);
  }

  for (int i = 90; i > 0; i--){
    dec_servo_H();
    dec_servo_V();
    delay(30);
  }
  
}


void inc_servo_H(void){
  current_angle_H += 1;
  SET_SERVO_H(current_angle_H);
}

void inc_servo_V(void){
  current_angle_V += 1;
  SET_SERVO_V(current_angle_V);
}
void dec_servo_H(void){
  current_angle_H -= 1;
  SET_SERVO_H(current_angle_H);
}

void dec_servo_V(void){
  current_angle_V -= 1;
  SET_SERVO_V(current_angle_V);
}
void SERVO_PWM_init(void){
  DDRB |= (1<<5);
  DDRB |= (1<<6);
  // DDRC |= (1<<5);
  // DDRC |= (1<<6);
  DDRD |= 1;
  
  TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11) | (0<<WGM10);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | 0b100;
  ICR1 = 1249;
  OCR1A = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
  OCR1B = ((0.5/20.0) + ((90/180.0) * (2))/20.0) * ICR1;
}
