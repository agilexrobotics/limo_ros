#ifndef LIMO_MSG_PARSER_H
#define LIMO_MSG_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame {
  uint32_t can_id;
  uint8_t can_dlc;
  uint8_t data[8] __attribute__((aligned(8)));
};
#endif

#include "limo_message.h"

bool DecodeCanFrame(const struct can_frame *rx_frame, AgxMessage *msg);
void EncodeCanFrame(const AgxMessage *msg, struct can_frame *tx_frame);
uint8_t CalcCanFrameChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif /* LIMO_MSG_PARSER_H */
