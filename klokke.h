#include <avr/pgmspace.h>
#define KLOKKESLETT_SIZE 10*24*2
#define PIN_CONVERT_SIZE 24*2
#define HS 60*60
#define MS 60
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet64(value, bit) ((value) |= (1ULL << (bit)))
#define bitClear64(value, bit) ((value) &= ~(1ULL << (bit)))
const uint32_t EOW = 43200;
const uint32_t DONETIME = 28800;
const uint32_t EOD = 86400;

const uint8_t klokkeslett[KLOKKESLETT_SIZE] PROGMEM = {
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

const uint8_t pin_convert_array[PIN_CONVERT_SIZE] PROGMEM = {
PIN_PA4, PIN_PA6, PIN_PG2, PIN_PC6, PIN_PC4, PIN_PC2, PIN_PC0, PIN_PG0, // r0 (CN1 on the silkscreen)
PIN_PA3, PIN_PA5, PIN_PA7, PIN_PC7, PIN_PC5, PIN_PC3, PIN_PC1, PIN_PG1, // r1 (CN2 on the silkscreen)
PIN_PD6, PIN_PD4, PIN_PD2, PIN_PG4, PIN_PB7, PIN_PB5, PIN_PB3, PIN_PB1, // r2 (CN3 on the silkscreen)
PIN_PD7, PIN_PD5, PIN_PD3, PIN_PD0, PIN_PG3, PIN_PB6, PIN_PB4, PIN_PB2, // r3 (CN4 on the silkscreen)
PIN_PE7, PIN_PE3, PIN_PF0, PIN_PF2, PIN_PF4, PIN_PF6, PIN_PA0, PIN_PA2, // r4 (CN5 on the silkscreen)
PIN_PB0, PIN_PE6, PIN_PE2, PIN_PF1, PIN_PF3, PIN_PF5, PIN_PF7, PIN_PA1  // r5 (CN6 on the silkscreen)
};

uint8_t get_pin(uint8_t row, uint8_t column) {
  return pgm_read_byte_near(pin_convert_array + (8*row) + column);
};

bool isNorwegianWinterTime(uint8_t month, uint8_t day, uint8_t day_of_week) {
    // Winter time is November to February
    if (month < 3 || month > 10) {
        return true;  // Winter time
    }

    // Summer time is April to September
    if (month > 3 && month < 10) {
        return false;  // Summer time
    }

    bool last_sunday_in_month_or_later;
    last_sunday_in_month_or_later = (day_of_week == 7) && (day > 24) ||
                                    (day_of_week == 1) && (day > 25) ||
                                    (day_of_week == 2) && (day > 26) ||
                                    (day_of_week == 3) && (day > 27) ||
                                    (day_of_week == 4) && (day > 28) ||
                                    (day_of_week == 5) && (day > 29) ||
                                    (day_of_week == 6) && (day > 30);

    if(month == 3) {
      return !last_sunday_in_month_or_later;
    }else {
      return last_sunday_in_month_or_later;
    }
}
