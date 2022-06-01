#define LATCH_PIN 8    //Pin connected to ST_CP of 74HC595
#define CLOCK_PIN 12   //Pin connected to SH_CP of 74HC595
#define DATA_PIN 11    //Pin connected to DS of 74HC595

#define STATUS_PIN 2
#define LBATT_PIN 3

void shiftOut(byte myDataOut) {
  digitalWrite(LATCH_PIN, 0);
  // This shifts 8 bits out MSB first
  uint8_t pinState;
  for (int8_t i=7; i>=0; i--)  {
    digitalWrite(CLOCK_PIN, 0);
    pinState = ( myDataOut & (1<<i) ) ? 1 : 0;
    digitalWrite(DATA_PIN, pinState);
    digitalWrite(CLOCK_PIN, 1);
  }
  digitalWrite(CLOCK_PIN, 0);
  digitalWrite(LATCH_PIN, 1);
}

void setupOutput(void){
  // Set up pin state
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STATUS_PIN, OUTPUT);
  for(uint8_t i=0;i<6;i++) {
    shiftOut(0x00);
  }
  pinMode(STATUS_PIN, OUTPUT);
  pinMode(LBATT_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);
  digitalWrite(LBATT_PIN,HIGH);
  delay(500);
  digitalWrite(STATUS_PIN, LOW);
  digitalWrite(LBATT_PIN, LOW);
}