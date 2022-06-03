#define LBATT_PIN 19
#define N_CLOCKS 48

const uint8_t pins[N_CLOCKS] PROGMEM = {
// 0
9,  8,  7,  6,  5,  4,  3,  2,
17, 16, 15, 14, 13, 12, 11, 10,
29, 28, 27, 26, 25, 24, 23, 22, 
37, 36, 35, 34, 33, 32, 31, 30,
45, 44, 43, 42, 41, 40, 39, 38,
53, 52, 51, 50, 49, 48, 47, 46
};

void turnOffPin(uint8_t pin_number){
  pinMode(pin_number, INPUT);
}

void turnOnPin(uint8_t pin_number){
  pinMode(pin_number, OUTPUT);
  digitalWrite(pin_number, LOW);
}

void setupOutput(void){
  for(uint8_t i = 0; i < N_CLOCKS; i++) {
    turnOffPin(pins[i]);
  }
}

void shiftOut(byte myDataOut, uint8_t latchon, uint8_t row) {
  for(uint8_t i = 0; i < 8; i++) {
    if(bitRead(myDataOut, i) == 1) {
      turnOnPin(pins[row * 8 + i]);
    } else {
      turnOffPin(pins[row * 8 + i]);
    }
  }
}