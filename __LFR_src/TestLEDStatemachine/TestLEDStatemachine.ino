#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
#define TGLBIT(x,y) x ^= BIT(y)

#define LED_1_DDR DDRB
#define LED_2_DDR DDRB
#define LED_3_DDR DDRD
#define LED_4_DDR DDRD
#define LED_5_DDR DDRD

#define LED_1_PORT PORTB
#define LED_2_PORT PORTB
#define LED_3_PORT PORTD
#define LED_4_PORT PORTD
#define LED_5_PORT PORTD

#define LED_1_PIN PIN3
#define LED_2_PIN PIN7
#define LED_3_PIN PIN0
#define LED_4_PIN PIN1
#define LED_5_PIN PIN2

void LED_init(void);

void setup() {
  // put your setup code here, to run once:
  // Serial.begin(9600);
  LED_init();
  SETBIT(LED_1_PORT, LED_1_PIN);
  SETBIT(LED_2_PORT, LED_2_PIN);
  SETBIT(LED_3_PORT, LED_3_PIN);
  SETBIT(LED_4_PORT, LED_4_PIN);
  SETBIT(LED_5_PORT, LED_5_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  CLEAR_LEDS();
  SETBIT(LED_1_PORT, LED_1_PIN);
  delay(1000);
  CLEAR_LEDS();
  SETBIT(LED_2_PORT, LED_2_PIN);
  delay(1000);
  CLEAR_LEDS();
  SETBIT(LED_3_PORT, LED_3_PIN);
  delay(1000);
  CLEAR_LEDS();
  SETBIT(LED_4_PORT, LED_4_PIN);
  delay(1000);
  CLEAR_LEDS();
  SETBIT(LED_5_PORT, LED_5_PIN);
}

void LED_init(void){
  // Set as output
  SETBIT(LED_1_DDR, LED_1_PIN);
  SETBIT(LED_2_DDR, LED_2_PIN);
  SETBIT(LED_3_DDR, LED_3_PIN);
  SETBIT(LED_4_DDR, LED_4_PIN);
  SETBIT(LED_5_DDR, LED_5_PIN);
  CLEAR_LEDS();
}

void CLEAR_LEDS(void){
  // Set to off
  CLRBIT(LED_1_PORT, LED_1_PIN);
  CLRBIT(LED_2_PORT, LED_2_PIN);
  CLRBIT(LED_3_PORT, LED_3_PIN);
  CLRBIT(LED_5_PORT, LED_5_PIN);
  CLRBIT(LED_4_PORT, LED_4_PIN);
}

void SET_ONE_LED(int PORT, int PIN){
  CLEAR_LEDS();
  SETBIT(PORT, PIN);
}

void SET_MULTI_LED(int PORT1, int PIN_1, int PORT2, int PIN_2){
  CLEAR_LEDS();
  SETBIT(PORT1, PIN_1);
  SETBIT(PORT2, PIN_2);
}