#ifndef LIMO_DATA_H
#define LIMO_DATA_H

#include <stdint.h>

#define LIMO_SYNC1	    0x55
#define LIMO_SYNC2	    0x0E

typedef struct {
  uint8_t state{0};
  uint8_t type;
  uint16_t can_id;
  uint8_t count;
  uint8_t data[8];
  uint16_t length;
  uint8_t RxCK;  // Rx check
  uint8_t hight;
  uint8_t low;
} LIMO_t_RAW_t;

enum {
  LIMO_WAIT_SYNC1 = 0,
  LIMO_WAIT_SYNC2,
  LIMO_WAIT_ID_HIGH,
  LIMO_WAIT_ID_LOW,
  LIMO_WAIT_DATA,
  LIMO_COUNT,
  LIMO_CHECK,
};

#endif  // LIMO_DATA_H