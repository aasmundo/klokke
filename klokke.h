#ifndef KLOKKE_H
#define KLOKKE_H
#include <avr/pgmspace.h>
#define FLASH_ARRAY_SIZE (10*24*2)
#define HS (60*60)
#define MS (60)
#define MIN_IN_DAY (24*60) // Minutes in a day
#define ENDTIME (7*60) // Time the watches must be finished
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet64(value, bit) ((value) |= (1ULL << (bit)))
#define bitClear64(value, bit) ((value) &= ~(1ULL << (bit)))

#define SLEEPBUFFER 2  //Sleep margin. Lowest safe value is 2

// Debug options
#define DBG_TIME_CALC 1
#define DBG_OUTPUTS   2
#define DBG_SLEEP     4
#define DBG_VOLTAGE   8

// Settings
uint8_t debug = DBG_TIME_CALC | DBG_OUTPUTS | DBG_SLEEP; // Write more stuff to the serial port
uint8_t reset_eeprom = 1; // Reset the clock states if first boot after programming
uint8_t startDay = 0;    // The number that the thing shows now
uint8_t fast = 0;         // Don't use RTC, but instead progress as fast as possible. For debug.

const uint8_t klokkeslett[FLASH_ARRAY_SIZE] PROGMEM = {
// 0
6,15,  2,45,  2,45,  5,45,
6,00,  6,15,  5,45,  6,00,
6,00,  6,00,  6,00,  6,00,
6,00,  6,00,  6,00,  6,00,
6,00,  3,00,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00,

//1
6,15,  2,45,  5,45,  7,38,
3,00,  5,45,  6,00,  7,38,
7,38,  6,00,  6,00,  7,38,
7,38,  6,00,  6,00,  7,38,
6,15,  9,00,  3,00,  5,45,
3,00,  2,45,  2,45,  9,00,
//2
6,15,  2,45,  2,45,  5,45,
3,00,  2,45,  5,45,  6,00,
6,15,  2,45,  9,00,  6,00,
6,00,  6,15,  2,45,  9,00,
6,00,  3,00,  2,45,  5,45,
3,00,  2,45,  2,45,  9,00,
//3
6,15,  2,45,  2,45,  5,45,
3,00,  2,45,  5,45,  6,00,
7,38,  6,15,  9,00,  6,00,
7,38,  3,00,  5,45,  6,00,
6,15,  2,45,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00,
//4
6,15,  5,45,  6,15,  5,45,
6,00,  6,00,  6,00,  6,00,
6,00,  3,00,  9,00,  6,00,
3,00,  2,45,  5,45,  6,00,
7,38,  7,38,  6,00,  6,00,
7,38,  7,38,  3,00,  9,00,
//5
6,15,  2,45,  2,45,  5,45,
6,00,  6,15,  2,45,  9,00,
6,00,  3,00,  2,45,  5,45,
3,00,  2,45,  5,45,  6,00,
6,15,  2,45,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00,
//6
6,15,  2,45,  2,45,  5,45,
6,00,  6,15,  2,45,  9,00,
6,00,  3,00,  2,45,  5,45,
6,00,  6,15,  5,45,  6,00,
6,00,  3,00,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00,
//7
6,15,  2,45,  2,45,  5,45,
3,00,  2,45,  5,45,  6,00,
7,38,  7,38,  6,00,  6,00,
7,38,  7,38,  6,00,  6,00,
7,38,  7,38,  6,00,  6,00,
7,38,  7,38,  3,00,  9,00,

//8
6,15,  2,45,  2,45,  5,45,
6,00,  6,15,  5,45,  6,00,
6,00,  3,00,  9,00,  6,00,
6,00,  6,15,  5,45,  6,00,
6,00,  3,00,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00,
//9
6,15,  2,45,  2,45,  5,45,
6,00,  6,15,  5,45,  6,00,
6,00,  3,00,  9,00,  6,00,
3,00,  2,45,  5,45,  6,00,
6,15,  2,45,  9,00,  6,00,
3,00,  2,45,  2,45,  9,00
};


int16_t getDest(uint8_t watch) {
  uint8_t hour = pgm_read_byte_near(klokkeslett + (2*watch));
  uint8_t minute = pgm_read_byte_near(klokkeslett + (2*watch) + 1);
  int16_t dest_min = 60*((int16_t) hour) + ((int16_t) minute);
  return dest_min;
  };

#endif