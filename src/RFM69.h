#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RFM69_FIFO_SIZE 66

typedef struct {
  uint8_t length;
  uint8_t dst;
  uint8_t src;
  struct {
    uint8_t req_ack : 1;
    uint8_t seq : 7;
  } flags;
} RFM69_Header;

typedef struct {
  RFM69_Header hdr;
  uint8_t payload[RFM69_FIFO_SIZE - sizeof(RFM69_Header)];
} RFM69_Packet;

typedef enum {
  RFM69_CMD_ACK,
  RFM69_CMD_MEASUREMENT,
} RFM69_CMD;

void RFM69_init(uint8_t node_id);
void RFM69_write_blocking(uint8_t addr, uint8_t val);
void RFM69_write16(uint8_t addr, uint16_t val);
uint8_t RFM69_read_blocking(uint8_t addr);
RFM69_Packet* RFM69_read_packet();
void RFM69_rxbuf_return();
uint8_t* RFM69_get_tx_payload();
void RFM69_send_packet(uint8_t dst, bool require_ack, uint8_t payload_len);
