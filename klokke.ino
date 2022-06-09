#ifdef ARDUINO_AVR_MEGA2560
#define PIN_OUTPUT
#define DS1307
#endif

// TODO: 
// Daylight saving time: https://www.instructables.com/Adding-Daylight-Savings-Time-to-Your-RTC/



#include "RTClib.h"

#include <avr/power.h>
#include "klokke.h"
#include <EEPROM.h>
#include <LowPower.h>
#include <Vcc.h>

#ifdef PIN_OUTPUT
#include "pin_output.h"
#else
#include "shift_output.h"
#endif

#ifdef DS1307
RTC_DS1307 rtc;
#else
RTC_DS3231 rtc;
#endif

int32_t unixtime_min = 0;

int16_t clocks_time[48] = {0};
byte clocks_last_state[6] = {0};

uint8_t get_secs_to_new_minute() {
  uint64_t tid = millis();
  DateTime now = rtc.now();
  if((millis() - tid) > 500) { // RTC most likely did not respond
    if(debug) {
      Serial.print(F("RTC timeout."));
    }
    delay(100);
    return 0;
  } else {
    if(fast==1) {
      return 0;
    }else{
      return (uint8_t) (60 - now.second());
    }
  }
}


// Sleep for until its soon a new minute
void sleep() {
  uint8_t secs_left = get_secs_to_new_minute();
  if(!(debug & DBG_SLEEP)){
    Serial.end();
    delay(10);
  }
  while(secs_left > SLEEPBUFFER) {
    if(debug & DBG_SLEEP){
      Serial.print(F("secs_left to new minute: "));
      Serial.println(secs_left);
      Serial.flush();
    }
    if(secs_left >= 16) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    } else if(secs_left >= 8) {
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    } else {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    }
    secs_left = get_secs_to_new_minute();
  }
  if(!(debug & DBG_SLEEP)){
    delay(20);
    Serial.begin(115200);
    delay(100);
  }
}



int32_t get_now() {
  uint64_t tid = millis();
  DateTime now = rtc.now();
  if((millis() - tid) > 500) { // RTC most likely did not respond
    if(debug) {
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

int32_t wait_new_minute() { // Wait to next minute
  sleep();            // Sleep almost a minute
  int32_t prev_minute, new_minute;
  do{
    prev_minute = get_now();
    if(debug & DBG_SLEEP){
      Serial.print(F("prev_minute: "));
      Serial.println(prev_minute);
    }
  } while(prev_minute == -1);
  do{
    new_minute = get_now();
  } while(new_minute == prev_minute || new_minute == -1);
  if(debug & DBG_SLEEP){
    Serial.print(F("new_minute: "));
    Serial.println(new_minute);
  }
  return new_minute;
}

uint8_t getTomorrow(DateTime now) {
  DateTime tomorrow(now + TimeSpan(1,0,0,0));
  return tomorrow.day();
}

int32_t getTomorrow(uint8_t day, uint8_t month, int32_t year) {
  uint8_t leap_year = 0;
  if ((year % 4 == 0 && year % 100 != 0) || year % 400 == 0) {
    leap_year = 1;
  }
  uint8_t monthLength[12];
  monthLength[0] = 31;
  monthLength[1] = leap_year == 1 ? 29 : 28;
  monthLength[2] = 31;
  monthLength[3] = 30;
  monthLength[4] = 31;
  monthLength[5] = 30;
  monthLength[6] = 31;
  monthLength[7] = 31;
  monthLength[8] = 30;
  monthLength[9] = 31;
  monthLength[10] = 30;
  monthLength[11] = 31;
  day += 1;
  if(day > monthLength[month-1]) {
    day = 1;
  }
  return (int32_t) day;
}

int32_t get_day() { // Get day of the month
  DateTime now = rtc.now();
  if(now.day() >31) {
    return 99;
  }else {
    if(fast==1) {
      return (int32_t) ((unixtime_min/MIN_IN_DAY)%32);
    } else {
      int32_t result = (now.hour() > (ENDTIME / 60)) ? getTomorrow(now) : ((int32_t) now.day());
      return result;
    }
  }
}


void write_clock_states(uint8_t day) { // Shift out on/off to all the clocks
    EEPROM.write(0,day);
    EEPROM.write(10,reset_eeprom);
}

void setup() {

  Serial.begin(115200);
  setupOutput();
  rtc.begin();

  if(debug) {
    Serial.println(F("Starting"));
    DateTime now = rtc.now();
    Serial.print(now.day());
    Serial.print(F("/"));
    Serial.print(now.month());
    Serial.print(F("/"));
    Serial.print(now.year());
    Serial.print(F(" "));
    Serial.print(now.hour());
    Serial.print(F(":"));
    Serial.print(now.minute());
    Serial.print(F(":"));
    Serial.println(now.second());

    if(now.day() == 0){
      Serial.println(F("RTC time not set!"));
      digitalWrite(LBATT_PIN,HIGH);
      delay(2000);
      digitalWrite(LBATT_PIN, LOW);
    }
  }

  // Reset the EEPROM
  if (reset_eeprom != EEPROM.read(10)) {
    write_clock_states(startDay);
    Serial.println(F("Loaded new day in to EEPROM"));
  }

  // Read the clock states stored in EEPROM
  startDay = EEPROM.read(0);
  Serial.print(F("Start day: "));
  Serial.println(startDay);
  uint8_t day[2];
  int8_t k;
  day[0] = startDay / 10;
  day[1] = startDay % 10;
  for(int8_t i=0;i<6;i++) {
    for(int8_t j=0;j<8;j++) {
      k = i*8 + j;
      clocks_time[k] = getDest((24*day[j/4]+((4*(5-i))+(j%4))));
    }
  }
}

// Calculate how many minutes a clock must run to reach
// the goal state
int16_t calc_runtime(int16_t state_now, int16_t goal_state) {
  if(state_now == goal_state) {
    return 0;
  } else if(state_now < goal_state) {
    return goal_state - state_now;
  } else {
    return goal_state + (720-state_now); // 720 minutes is how many minutes before clock wraps
  }
}

// Calculate when the clock must start running
// to finish its runtime at the given endtime
int16_t calc_start_time(int16_t runtime) {
  int16_t start_time = ENDTIME - runtime;
  start_time = start_time < 0 ? start_time + 1440 : start_time;
  return start_time;
}

// Main turn on/off clocks function
void update_states() {
  int8_t k;
  int32_t dom = get_day(); // Day of the month
  int16_t goal_state;      // The goal clock state for a clock
  int16_t starttime;       // The time a clock must start to reach goal state
  uint8_t day[2];          // The two numbers displayed;
  uint8_t diff = 0;        // If there is a diff, one or more clocks needs to turn on
  day[0] = dom / 10;
  day[1] = dom % 10;
  if(debug) {
    Serial.print(F("Target day: "));
    Serial.print(day[0]);
    Serial.println(day[1]);
    // Print the unixtime in milliseconds for offline simulation
    Serial.print(F("Unixtime in ms: "));
    Serial.print(((uint32_t) (unixtime_min%MIN_IN_DAY))*6);
    Serial.println(F("0000,"));
  }

  for(int8_t i=0;i<6;i++) {
    byte start_byte = 0;
    for(int8_t j=0;j<8;j++) {
      k = i*8 + j;
      // If the clock was on the last minute, update the state
      if(bitRead(clocks_last_state[i], j)) {
        clocks_time[k] = (clocks_time[k] + 1) % 720;
      }
      // Find the goal state for the clock
      goal_state = getDest((24*day[j/4]+((4*(5-i))+(j%4))));
      // Find the starttime of this clock given the goal state and the endtime
      starttime = calc_start_time(calc_runtime(clocks_time[k],goal_state));
      if (debug & DBG_TIME_CALC) {
        Serial.print(F("goal_state, clocks_time, startime, unixtime%1440: "));
        Serial.print(goal_state);
        Serial.print(F(", "));
        Serial.print(clocks_time[k]);
        Serial.print(F(", "));
        Serial.print(starttime);
        Serial.print(F(", "));
        Serial.println(unixtime_min % MIN_IN_DAY);
      }
      // Check if clock needs to start now
      if ((unixtime_min % MIN_IN_DAY) == starttime) { // Start the clock if time is "starttime"
        bitSet(start_byte,j);
      }
    }
    // Only start the ones that are not already running
    //start_byte = clocks_last_state[i] ^ start_byte;
    // If a clock needs to start, set diff flag
    diff += start_byte != clocks_last_state[i] ? 1 : 0;
    if(debug & DBG_OUTPUTS){
      Serial.print(F("start_byte, clocks_last_state[i], diff: "));
      Serial.print(start_byte, BIN);
      Serial.print(F(", "));
      Serial.print(clocks_last_state[i], BIN);
      Serial.print(F(", "));
      Serial.println(diff);
    }
    // Turn on the clocks that needs to start
    clocks_last_state[i] |= start_byte;
    // If the end time has been reached. Force the diff flag
    // and turn off all clocks
    if ((unixtime_min % MIN_IN_DAY) == ENDTIME) {
      diff = 1;
      clocks_last_state[i] = 0;
    }

  }
  // Shift out the states to the shift registers
  // that power the clocks
  if(diff != 0) {
    for(uint8_t i=0;i<6;i++) {
      if(debug & DBG_OUTPUTS) {
        for(int8_t j=7;j>=0;j--) {
          Serial.print(bitRead(clocks_last_state[i],j));
          Serial.print(F(","));
        }
        Serial.println();
      }
      shiftOut(clocks_last_state[i], i == 5, i);
    }
  }
  if(debug) {
    Serial.println(F("NEXT"));
  }
}

// Read the VCC voltage and drive the battery status LEDs
void low_battery() {
  long mv_batt = Vcc::measure(10,1100);
  if(mv_batt < 3300 ) {
    digitalWrite(LBATT_PIN, HIGH);
  } else {
    digitalWrite(LBATT_PIN, LOW);
  }
  Serial.print(F("MCU voltage: "));
  Serial.println((uint32_t) mv_batt);
#ifdef PIN_OUTPUT
  readVoltages();
#endif
}

void loop() {
  
  // Wait for the next minute
  unixtime_min = wait_new_minute();
  // Update the clock states
  update_states();
  Serial.print(F("unixtime_min: "));
  Serial.println(unixtime_min);
  Serial.print(F("unixtime_min_day: "));
  Serial.print((unixtime_min % 1440) / 60);
  Serial.print(F(":"));
  Serial.println((unixtime_min % 1440) % 60);
  if(fast==0){ // Skip sleep and EEPROM write if in fast mode
    low_battery(); // Check battery level
    if((unixtime_min % MIN_IN_DAY) == ENDTIME) {
      write_clock_states(((uint8_t) get_day())); // Write clock states to EEPROM so power can be removed
    }

  }


}
