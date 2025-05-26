#define BIT(x) (1<<(x))
#define SETBIT(x,y) ((x) |= BIT(y))
#define CLRBIT(x,y) ((x) &= ~BIT(y))
#define TGLBIT(x,y) x ^= BIT(y)

void setup() {
  SETBIT(DDRB, PIN7);      // set LED pin as output
  SETBIT(PORTB, PIN7);    // switch off LED pin

  Serial1.begin(9600);            // initialize UART with baud rate of 9600
}
void loop() {
  while (Serial1.available() >= 0) {
    char receivedData = Serial1.read();   // read one byte from serial buffer and save to receivedData
    if (receivedData == '1') {
      SETBIT(DDRB, PIN7); // switch LED On
    }
    else if (receivedData == '0') {
      CLRBIT(DDRB, PIN7);  // switch LED Off
    }
  }
  // Serial1.write(1);
}


