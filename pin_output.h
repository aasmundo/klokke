#include "klokke.h"
#define TEMP_PIN 18
#define LBATT_PIN 19
#define BATT_MEAS_PIN A0
#define RTC_BATT_PIN A1
#define CLOCK_VOLTAGE_PIN A2
#define N_CLOCKS 48

// Seen from the back
const uint8_t pins[N_CLOCKS] = {
2,  3,  4,  5,  6,  7,  8,  9,
10, 11, 12, 13, 14, 15, 16, 17, 
22, 23, 24, 25, 26, 27, 28, 29,
30, 31, 32, 33, 34, 35, 36, 37,
38, 39, 40, 41, 42, 43, 44, 45,
46, 47, 48, 49, 50, 51, 52, 53
};


// // Seen from the front
// const uint8_t pins[N_CLOCKS] = {
// 9,  8,  7,  6,  5,  4,  3,  2,
// 17, 16, 15, 14, 13, 12, 11, 10,
// 29, 28, 27, 26, 25, 24, 23, 22, 
// 37, 36, 35, 34, 33, 32, 31, 30,
// 45, 44, 43, 42, 41, 40, 39, 38,
// 53, 52, 51, 50, 49, 48, 47, 46
// };


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
  pinMode(LBATT_PIN, OUTPUT);
  digitalWrite(LBATT_PIN,HIGH);
  delay(100);
  digitalWrite(LBATT_PIN, LOW);

  // Also setup some inputs
  pinMode(BATT_MEAS_PIN, INPUT);
  pinMode(RTC_BATT_PIN, INPUT);
  pinMode(CLOCK_VOLTAGE_PIN, INPUT);
}

long readAnalogVoltage(uint8_t pin, uint8_t measurements){
  long voltage = 0;
  for(uint8_t i=0; i<measurements; i++){
    voltage += (long)analogRead(pin);
  }
  voltage = ((voltage / measurements) * 5000) / 1024;
  return voltage;
}

void readVoltages(void) {
  long battery_voltage = readAnalogVoltage(BATT_MEAS_PIN,10);
  long rtc_voltage = readAnalogVoltage(RTC_BATT_PIN,10);
  long clock_voltage = readAnalogVoltage(CLOCK_VOLTAGE_PIN,10);
  if(debug & DBG_VOLTAGE) {
    Serial.print(F("Battery voltage: "));
    Serial.println(battery_voltage);
    Serial.print(F("RTC battery voltage: "));
    Serial.println(rtc_voltage);
    Serial.print(F("Clock voltage: "));
    Serial.println(clock_voltage);
  }
  if(battery_voltage < 3300 && battery_voltage > 1000){
    digitalWrite(LBATT_PIN,HIGH);
    delay(200);
    digitalWrite(LBATT_PIN, LOW);
  }
}

void shiftOut(byte myDataOut, uint8_t latchon, uint8_t row) {
  for(uint8_t i = 0; i < 8; i++) {
    uint8_t pin_number = pins[(N_CLOCKS -1) - (row * 8 + i)];
    if(bitRead(myDataOut, i) == 1) {
      turnOnPin(pin_number);
      if(debug & DBG_OUTPUTS){
        Serial.print(F("Turning on pin "));
        Serial.println(pin_number);
      }
    } else {
      turnOffPin(pin_number);
    }
  }
}
