#include <WSWire.h>
#include <RTClib.h>
#include "klokke.h"
#include <EEPROM.h>
#include <LowPower.h>
RTC_DS1307 RTC;


uint8_t rtc_power_pin = 9;

void turn_on_RTC() {
  digitalWrite(rtc_power_pin, HIGH);
}

void turn_off_RTC() {
  digitalWrite(rtc_power_pin, LOW);
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
  turn_off_RTC();
  Serial.end();
  delay(10);
  float duration_f = ((float) duration) - 1.0;
  while(duration > 5) {
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    duration = duration - 4.05;
  }
  LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);
  delay(20);
  Serial.begin(115200);
  turn_on_RTC();
  delay(100);
}

int32_t unixtime_min = 0;

int16_t clocks_time[48] = {0};
int16_t clocks_runtime[48] = {0};
byte clocks_last_state[6] = {0};
uint8_t debug = 1;
uint8_t fast = 1;
uint8_t day = 0;


int32_t get_now() {
  uint64_t tid = millis();
  DateTime now = RTC.now();
  if((millis() - tid) > 500) { // RTC most likely did not respond
    if(debug==1) {
      Serial.print(F("RTC timout, starter RTC om igjen."));
    }
    turn_on_RTC();
    delay(100);
    return -1;
  } else {
    if(fast==1) {
      return (int32_t) (millis() / 100); 
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
  if(now.day() < 0 || now.day() >31) {
    return 99;
  }else {
    if(fast==1) {
      return (int32_t) ((unixtime_min/1440)%32);
    } else {
      return (int32_t) now.day();
    }
  }
}


void write_clock_states() {
  uint16_t temp16;
  byte tempbyte;
  for(uint8_t i=0;i<48;i++) {
    temp16 = (uint16_t) clocks_runtime[i];
    tempbyte = temp16 / 256;
    EEPROM.write((i*2),tempbyte);
    tempbyte = temp16;
    EEPROM.write((i*2)+1,tempbyte);
  }
}

void setup() {
  pinMode(8, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(2, INPUT_PULLUP);

  pinMode(rtc_power_pin, OUTPUT);

  turn_on_RTC();

  Wire.begin();
  RTC.begin();
  RTC.adjust(DateTime(18,8,5,23,59,56));
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
  for(uint8_t i=0;i<48;i++) {
    temp_a = (int16_t) EEPROM.read(i*2);
    temp_b = (int16_t) EEPROM.read((i*2)+1);
    clock_states_temp = (temp_a*256) + temp_b;
    if(debug == 1) {
      Serial.println(clock_states_temp);
      EEPROM.write(i*2,i);
      EEPROM.write((i*2)+1,i);
    } else {
      clocks_runtime[i] = clock_states_temp;
    }
  }
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

void update_states(uint8_t low_batt) {
  int8_t k;
  int32_t dom = get_day();
  int16_t goal_state;
  int16_t starttime;
  uint8_t day[2];
  day[0] = dom / 10;
  day[1] = dom % 10;
  if(debug==1) {
    Serial.print(((uint32_t) (unixtime_min%1440))*6);
    Serial.print("0000,");
  }
  byte shift_byte;
  for(int8_t i=0;i<6;i++) {
    for(int8_t j=0;j<8;j++) {
      k = i*8 + j;
      uint8_t l = 7-j;
      if(bitRead(clocks_last_state[i], l)) {
        clocks_time[k] = (clocks_time[k] + 1) % 720;
      }
      
      goal_state = getDest((24*day[k/24])+(k%24));
      starttime = calc_start_time(calc_runtime(clocks_time[k],goal_state));
      if ((unixtime_min % 1440) == starttime && ((unixtime_min % 1440) < 720)) {
        bitSet(shift_byte,l);
      } else if(((unixtime_min % 1440) > starttime) && ((unixtime_min % 1440) < 720)) {
        bitSet(shift_byte,l);
      } else{
        bitClear(shift_byte,l);
      }
    }
    clocks_last_state[i] = shift_byte;
    if(debug==1) {
        for(int8_t j=7;j>=0;j--) {
          Serial.print(bitRead(shift_byte,j));
          Serial.print(",");
        }
      } else {
        shiftOut(shift_byte);
      }
  }
  if(debug==1) {
    Serial.println(F("NEXT"));
  }
}

uint8_t low_battery() {
  // TODO
  return 0;
}

void loop() {
  unixtime_min = wait_new_minute();
  update_states(0);
  if(fast==0){
    if((unixtime_min % 1440) == 720) {
      write_clock_states();
      sleep(40000);
    }
  
      sleep(57);
  }


}