#include <Wire.h>
#include <DS3231.h>
#include <avr/power.h>

#include "klokke.h"
#include <EEPROM.h>
#include <LowPower.h>


#define MIN_IN_DAY 1440
#define STATUS_PIN 2
#define LBATT_PIN 3

RTClib RTC;




long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


void shiftOut(byte myDataOut) {
  //Pin connected to ST_CP of 74HC595
  int latchPin = 8;
  //Pin connected to SH_CP of 74HC595
  int clockPin = 12;
  ////Pin connected to DS of 74HC595
  int dataPin = 11;
  digitalWrite(latchPin, 0);
  // This shifts 8 bits out MSB first
  int i=0;
  int pinState;
  for (i=7; i>=0; i--)  {
    digitalWrite(clockPin, 0);
    if ( myDataOut & (1<<i) ) {
      pinState= 1;
    }
    else {  
      pinState= 0;
    }
    digitalWrite(dataPin, pinState);
    digitalWrite(clockPin, 1);
    digitalWrite(dataPin, 0);
  }
  digitalWrite(clockPin, 0);
  digitalWrite(latchPin, 1);
}

void sleep(uint16_t duration) {
  if (duration == 0) {
    return;
  }
  Serial.end();
  delay(10);
  while(duration > 5) {
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    duration = duration - 4.05;
  }
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
  delay(20);
  Serial.begin(115200);
  delay(100);
}


int32_t unixtime_min = 0;

int16_t clocks_time[48] = {0};
byte clocks_last_state[6] = {0};
uint8_t debug = 1;
uint8_t reset_eeprom = 1;
uint8_t fast = 0;


int32_t get_now() {
  uint64_t tid = millis();
  DateTime now = RTC.now();
  if((millis() - tid) > 500) { // RTC most likely did not respond
    if(debug==1) {
      Serial.print(F("RTC timeout."));
    }
    delay(100);
    return -1;
  } else {
    if(fast==1) {
      return (int32_t) (unixtime_min + random(0,2)); 
    }else{
      return (int32_t) (now.unixtime() / 60);      
    }
  }
}

int32_t wait_new_minute() {
  int32_t prev_minute, new_minute;
  do{
    prev_minute = get_now();
  } while(prev_minute == -1);
  do{
    new_minute = get_now();
    
  } while(new_minute == prev_minute || new_minute == -1);
  return new_minute;
}

int32_t get_day() {
  DateTime now = RTC.now();
  if(now.day() >31) {
    return 99;
  }else {
    if(fast==1) {
      return (int32_t) ((unixtime_min/MIN_IN_DAY)%32);
    } else {
      return (int32_t) now.day();
    }
  }
}


void write_clock_states() {
  uint16_t temp16;
  byte tempbyte;
  for(uint8_t i=0;i<48;i++) {
    temp16 = (uint16_t) clocks_time[i];
    tempbyte = temp16 / 256;
    EEPROM.write((i*2),tempbyte);
    tempbyte = temp16;
    EEPROM.write((i*2)+1,tempbyte);
  }
}

void setup() {
  clock_prescale_set(clock_div_2);
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(2, INPUT_PULLUP);
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


  Wire.begin();

  int16_t temp_a;
  int16_t temp_b;
  int16_t clock_states_temp;

  if(debug==1) {
    Serial.begin(115200);
    Serial.println(F("starter"));
    DateTime now = RTC.now();
    Serial.print(now.day());
    Serial.print("/");
    Serial.print(now.month());
    Serial.print("/");
    Serial.print(now.year());
    Serial.print(" ");
    Serial.print(now.hour());
    Serial.print(":");
    Serial.print(now.minute());
    Serial.print(":");
    Serial.println(now.second());
  }

  if (reset_eeprom == 1) {
    write_clock_states();
  }

  
  for(uint8_t i=0;i<48;i++) {
    temp_a = (int16_t) EEPROM.read(i*2);
    temp_b = (int16_t) EEPROM.read((i*2)+1);
    clock_states_temp = (temp_a*256) + temp_b;
    if(debug == 1) {
      Serial.println(clock_states_temp);
      EEPROM.write(i*2,i);
      EEPROM.write((i*2)+1,i);
    } else {
    clocks_time[i] = clock_states_temp;
    }
  }
  low_battery();
}

int16_t calc_runtime(int16_t state_now, int16_t goal_state) {
  if(state_now == goal_state) {
    return 0;
  } else if(state_now < goal_state) {
    return goal_state - state_now;
  } else {
    return goal_state + (720-state_now);
  }
}

int16_t calc_start_time(int16_t runtime) {
  return 720 - runtime;
}

void update_states() {
  int8_t k;
  int32_t dom = get_day();
  int16_t goal_state;
  int16_t starttime;
  uint8_t day[2];
  uint8_t diff = 0;
  day[0] = dom / 10;
  day[1] = dom % 10;
  if(debug==1) {
    Serial.print(((uint32_t) (unixtime_min%MIN_IN_DAY))*6);
    Serial.print("0000,");
  }
  
  for(int8_t i=0;i<6;i++) {
    byte shift_byte = 0;
    for(int8_t j=0;j<8;j++) {
      k = i*8 + j;
      uint8_t l = 7-j;
      if(bitRead(clocks_last_state[i], l)) {
        clocks_time[k] = (clocks_time[k] + 1) % 720;
      }
      
      goal_state = getDest((24*day[k/24])+(k%24));
      starttime = calc_start_time(calc_runtime(clocks_time[k],goal_state));
      if (debug == 1) {
        Serial.print("goal_state, startime, unixtime%1440: ");
        Serial.print(goal_state);
        Serial.print(", ");
        Serial.print(starttime);
        Serial.print(", ");
        Serial.println(unixtime_min % MIN_IN_DAY);
      }
      if ((unixtime_min % MIN_IN_DAY) == starttime && ((unixtime_min % MIN_IN_DAY) < 720)) {
        bitSet(shift_byte,l);
      } else if(((unixtime_min % MIN_IN_DAY) > starttime) && ((unixtime_min % MIN_IN_DAY) < 720)) {
        bitSet(shift_byte,l);
      } else{
        bitClear(shift_byte,l);
      }
    }
    diff += (clocks_last_state[i] != shift_byte) ? 1 : 0;
    clocks_last_state[i] = shift_byte;

  }
  if(diff != 0) {
    for(uint8_t i=0;i<6;i++) {
      if(debug==1) {
        for(int8_t j=7;j>=0;j--) {
          Serial.print(bitRead(clocks_last_state[i],j));
          Serial.print(",");
        }
      }
      shiftOut(clocks_last_state[i]);
    }
  }
  if(debug==1) {
    Serial.println(F("NEXT"));
  }
}

void low_battery() {
  long mv_batt = readVcc();
  if(mv_batt < 3275 && mv_batt >3200) {
    digitalWrite(STATUS_PIN, HIGH);
  } else if (mv_batt <= 3200) {
    digitalWrite(STATUS_PIN, HIGH);
    digitalWrite(LBATT_PIN, HIGH);
  } else {
    digitalWrite(STATUS_PIN, LOW);
    digitalWrite(LBATT_PIN, LOW);
  }
  Serial.print("Batt: ");
  Serial.println((uint32_t) mv_batt);
}

void loop() {
  unixtime_min = wait_new_minute();
  Serial.print("unixtime_min: ");
  Serial.println(unixtime_min);
  update_states();
  if(fast==0){
    if((unixtime_min % MIN_IN_DAY) == 720) {
      write_clock_states();
      sleep(40000);
    }
      low_battery();
      sleep(54);
  }


}
