#include <Wire.h>
#include <DS3231.h>
#include <avr/power.h>

#include "klokke.h"
#include <EEPROM.h>
#include <LowPower.h>

#define MIN_IN_DAY 1440// Minutes in a day
#define ENDTIME 360    // Time the watches must be finished
#define STATUS_PIN 2
#define LBATT_PIN 3
#define LATCH_PIN 8    //Pin connected to ST_CP of 74HC595
#define CLOCK_PIN 12   //Pin connected to SH_CP of 74HC595
#define DATA_PIN 11    //Pin connected to DS of 74HC595
#define SLEEPBUFFER 2  //Sleep margin

RTClib RTC;

int32_t unixtime_min = 0;

int16_t clocks_time[48] = {0};
byte clocks_last_state[6] = {0};
bool disable = false;  //Don't start the clocks (late start or low batt)


// Settings
uint8_t debug = 1;        // Write more stuff to the serial port
uint8_t reset_eeprom = 2; // Reset the clock states if first boot after programming
uint8_t startDay = 1;    // The number that the thing shows now
uint8_t fast = 0;         // Don't use RTC, but instead progress as fast as possible. For debug.


bool are_clocks_running() {
  for (uint8_t i=0; i < 6; i++) {
    if(clocks_last_state[i] != 0) {
      return true;
    }
  }
  return false;
}

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

uint8_t get_secs_to_new_minute() {
  uint64_t tid = millis();
  DateTime now = RTC.now();
  if((millis() - tid) > 500) { // RTC most likely did not respond
    if(debug==1) {
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

void shiftOut(byte myDataOut, uint8_t latchon) {
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
  if(latchon != 0) {
    digitalWrite(LATCH_PIN, 1);
  }
}

// Sleep for until its soon a new minute
void sleep() {
  uint8_t secs_left = get_secs_to_new_minute();
  Serial.end();
  delay(10);
  while(secs_left > SLEEPBUFFER) {
    if(secs_left >= 12) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    } else if(secs_left >= 8) {
      LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    } else {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);
    }
    secs_left = get_secs_to_new_minute();
  }
  delay(20);
  Serial.begin(115200);
  delay(100);
}



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

int32_t wait_new_minute() { // Wait to next minute
  sleep();            // Sleep almost a minute
  int32_t prev_minute, new_minute;
  do{
    prev_minute = get_now();
  } while(prev_minute == -1);
  do{
    new_minute = get_now();

  } while(new_minute == prev_minute || new_minute == -1);
  return new_minute;
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
  DateTime now = RTC.now();
  if(now.day() >31) {
    return 99;
  }else {
    if(fast==1) {
      return (int32_t) ((unixtime_min/MIN_IN_DAY)%32);
    } else {
      int32_t result = (now.hour() > (ENDTIME / 60)) ? getTomorrow(now.day(), now.month(), ((int32_t) now.year()) + 1900) : ((int32_t) now.day());
      return result;
    }
  }
}


void write_clock_states(uint8_t day) { // Shift out on/off to all the clocks
    EEPROM.write(0,day);
    EEPROM.write(10,reset_eeprom);
}

void setup() {
  // Set up pin state
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(STATUS_PIN, OUTPUT);
  for(uint8_t i=0;i<6;i++) {
    shiftOut(0x00, i == 5 );
  }
  pinMode(STATUS_PIN, OUTPUT);
  pinMode(LBATT_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);
  digitalWrite(LBATT_PIN,HIGH);
  delay(500);
  digitalWrite(STATUS_PIN, LOW);
  digitalWrite(LBATT_PIN, LOW);

  // Start I2C for RTC
  Wire.begin();

  if(debug==1) {
    Serial.begin(115200);
    Serial.println(F("Starting"));
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

  // Disable if powered on less than 12 hours before ENDTIME
  int32_t starttime = get_now() % 1440;
  if(starttime < ENDTIME || starttime > (ENDTIME + 720)) {
    disable = true;
  }

  // Reset the EEPROM
  if (reset_eeprom != EEPROM.read(10)) {
    write_clock_states(startDay);
    Serial.println(F("Loaded new day in to EEPROM"));
  }

  // Read the clock states stored in EEPROM
  startDay = EEPROM.read(0);
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
  if(debug==1) { // Print the unixtime in milliseconds for offline simulation
    Serial.print(((uint32_t) (unixtime_min%MIN_IN_DAY))*6);
    Serial.print("0000,");
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
      if (debug == 1) {
        Serial.print("goal_state, clocks_time, startime, unixtime%1440: ");
        Serial.print(goal_state);
        Serial.print(", ");
        Serial.print(clocks_time[k]);
        Serial.print(", ");
        Serial.print(starttime);
        Serial.print(", ");
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
    Serial.print("start_byte, clocks_last_state[i], diff: ");
    Serial.print(start_byte);
    Serial.print(", ");
    Serial.print(clocks_last_state[i]);
    Serial.print(", ");
    Serial.println(diff);
    // Turn on the clocks that needs to start
    clocks_last_state[i] |= start_byte;
    // If the end time has been reached. Force the diff flag
    // and turn off all clocks
    if (((unixtime_min % MIN_IN_DAY) == ENDTIME) || disable) {
      diff = disable ? 0 : 1;
      clocks_last_state[i] = 0;
    }

  }
  // Shift out the states to the shift registers
  // that power the clocks
  if(diff != 0) {
    for(uint8_t i=0;i<6;i++) {
      if(debug==1) {
        for(int8_t j=7;j>=0;j--) {
          Serial.print(bitRead(clocks_last_state[i],j));
          Serial.print(",");
        }
      }
      shiftOut(clocks_last_state[i], i == 5);
    }
  }
  if(debug==1) {
    Serial.println(F("NEXT"));
  }
}

// Read the VCC voltage and drive the battery status LEDs
void low_battery() {
  long mv_batt = readVcc();
  if(mv_batt < 3300 ) {
    digitalWrite(LBATT_PIN, HIGH);
    if(!are_clocks_running()) {
      disable = true;
    }
  } else {
    digitalWrite(LBATT_PIN, LOW);
  }
  Serial.print("Batt: ");
  Serial.println((uint32_t) mv_batt);
}

void loop() {
  
  // Wait for the next minute
  unixtime_min = wait_new_minute();
  // Update the clock states
  update_states();
  Serial.print("unixtime_min: ");
  Serial.println(unixtime_min);
  Serial.print("unixtime_min_day: ");
  Serial.print((unixtime_min % 1440) / 60);
  Serial.print(":");
  Serial.println((unixtime_min % 1440) % 60);
  if(fast==0){ // Skip sleep and EEPROM write if in fast mode
    low_battery(); // Check battery level
    if((unixtime_min % MIN_IN_DAY) == ENDTIME) {
      write_clock_states(((uint8_t) get_day())); // Write clock states to EEPROM so power can be removed
      disable = false;
    }

  }


}
