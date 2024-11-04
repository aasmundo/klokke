#include <DS3231.h>
#include <Wire.h>
#include <avr/sleep.h>

#include "klokke.h"
#include <EEPROM.h>

#define MIN_IN_DAY 1440// Minutes in a day
#define ENDTIME_W 420    // Time the watches must be finished (winter)
#define ENDTIME_S 360  // Time the watches must be finished during the summer
#define EVENINGTIME 1080 // 18:00
#define DLSTIME 500    // Time that daylight savings adjustment is checked
#define LBATTVOLTAGE 3450

DS3231 myRTC;
bool century = false;
bool h12 = false;
bool pm = false;


int64_t unixtime_min = 0;
uint8_t dayOfWeek = 0;
uint32_t ENDTIME = ENDTIME_W;

int16_t clocks_time[48] = {0};
byte clocks_last_state[6] = {0};
bool disable = false;  //Don't start the clocks (late start or low batt)


// Settings
uint8_t debug = 1;        // Write more stuff to the serial port
uint8_t reset_eeprom = 5; // Reset the clock states if first boot after programming
uint8_t startDay = 23;    // The number that the thing shows now
uint8_t fast = 0;         // Don't use RTC, but instead progress as fast as possible. For debug.


void enter_power_down_mode() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set power-down sleep mode
    sleep_enable();                       // Enable sleep mode

    // Go to sleep (CPU halts here until interrupt occurs)
    sleep_cpu();

    // Code resumes here after interrupt
    sleep_disable();  // Disable sleep mode on wakeup
}

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
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void reduce_power() {
  PRR  |= 1 << PRLCD;  // Disable atmega169 LCD circuitry
  PRR  |= 1 << PRTIM1; // Disable time1
  ACSR |= 1 << ACD;    // Disable analog comparator
}

void setup_alarm() {
    myRTC.turnOffAlarm(1);
    myRTC.setA1Time(0, 0, 0, 0, 0b1110, 1, 0, 0);
    // enable Alarm 1 interrupts
    myRTC.turnOnAlarm(1);
    // clear Alarm 1 flag
    myRTC.checkIfAlarm(1);
    
    // Upload the parameters to prevent Alarm 2 entirely
    myRTC.setA2Time(0, 0, 0, 0b01100000, 1, 0, 0);
    // disable Alarm 2 interrupt
    myRTC.turnOffAlarm(2);
    // clear Alarm 2 flag
    myRTC.checkIfAlarm(2);
    // attach clock interrupt
    pinMode(PIN_PD1, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_PD1), isr_wakeup, LOW); // Dont use "FALLING" because this requires clk
}

void apply_clock_states_row(uint8_t states, uint8_t row) {
  for(uint8_t column=0; column<8; column++) {
    uint8_t pin = get_pin(row, column);
    if(bitRead(states, column)) { // Turn the clock on by setting pin LOW
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    } else { // Turn the clock off by setting the pin to Z
      pinMode(pin, INPUT);
    }
  }
}

int64_t get_now() {
  DateTime now = DateTime(myRTC.getYear(), myRTC.getMonth(century), myRTC.getDate());
  if(fast==1) {
    return (int32_t) (unixtime_min + random(0,2));
  }else{
    return (int32_t) (now.unixtime() / 60);
  }
}

int64_t wait_new_minute() { // Wait to next minute
  int64_t prev_minute, new_minute;
  do{
    prev_minute = get_now();
  } while(prev_minute == -1);
  if(fast == 0) {
    enter_power_down_mode();
  }
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

void setEndTime() {
  int32_t day = getTomorrow(myRTC.getDate(), myRTC.getMonth(century), ((int32_t) myRTC.getYear()) + 2000);
  uint8_t DoW_tomorrow = myRTC.getDoW() == 7 ? 1 : myRTC.getDoW() + 1;
  bool winterTime = isNorwegianWinterTime(myRTC.getMonth(century), (uint8_t) day, DoW_tomorrow);
  ENDTIME = winterTime ? ENDTIME_W : ENDTIME_S;
}

int32_t get_day() { // Get day of the month
  if(myRTC.getDate() >31) {
    return 99;
  }else {
    if(fast==1) {
      return (int32_t) ((unixtime_min/MIN_IN_DAY)%32);
    } else {
      int32_t result = (myRTC.getHour(h12, pm) > (ENDTIME / 60)) ? getTomorrow(myRTC.getDate(), myRTC.getMonth(century), ((int32_t) myRTC.getYear()) + 2000) : ((int32_t) myRTC.getDate());
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
  for(uint8_t i=0;i<6;i++) {
    apply_clock_states_row(0x00, i == 5);
  }

  // Start I2C for RTC
  Wire.begin();
  setup_alarm();

  if(debug==1) {
    Serial.begin(115200);
    Serial.println(F("Starting"));
    Serial.print(myRTC.getDate());
    Serial.print("/");
    Serial.print(myRTC.getMonth(century));
    Serial.print("/");
    Serial.print(myRTC.getYear());
    Serial.print(" ");
    Serial.print(myRTC.getHour(h12, pm));
    Serial.print(":");
    Serial.print(myRTC.getMinute());
    Serial.print(":");
    Serial.print(myRTC.getSecond());
    Serial.print("   DoW:");
    Serial.println(myRTC.getDoW());
  }

  // Disable if powered on less than 12 hours before ENDTIME
  uint32_t starttime = get_now() % 1440;
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

  setEndTime();
  // Wait until next day in case power/reset in the evening
  do {
    unixtime_min = wait_new_minute();
  } while (((unixtime_min%MIN_IN_DAY) > EVENINGTIME) || ((unixtime_min%MIN_IN_DAY) < ENDTIME));
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
        Serial.println((int32_t) (unixtime_min % MIN_IN_DAY));
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
      apply_clock_states_row(clocks_last_state[i],i);
    }
  }
  if(debug==1) {
    Serial.println(F("NEXT"));
  }
}
bool is_battery_low() {
  long mv_batt = readVcc();
  return (mv_batt < LBATTVOLTAGE);
}

// Read the VCC voltage
void low_battery() {
  long mv_batt = readVcc();
  if(is_battery_low() && !are_clocks_running()) {
      disable = true;
  } else {
    disable = false;
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
  Serial.println((int32_t) (unixtime_min));
  Serial.print("unixtime_min_day: ");
  Serial.print((int32_t) ((unixtime_min % 1440) / 60));
  Serial.print(":");
  Serial.println((int32_t) ((unixtime_min % 1440) % 60));
  if(fast==0){ // Skip sleep and EEPROM write if in fast mode
    if((unixtime_min % MIN_IN_DAY) == ENDTIME) {
      if (disable == false) {
        write_clock_states(((uint8_t) get_day())); // Write clock states to EEPROM so power can be removed
        setEndTime();
      }
      low_battery(); // Check battery level
      disable = false;
    }
  }
}

void isr_wakeup() {
    myRTC.checkIfAlarm(1); // Disables this alarm
    return;
}
