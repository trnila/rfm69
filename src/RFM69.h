#pragma once

#include <stdint.h>

#define RFM69_REG_FIFO 0x00
#define RFM69_REG_OPMODE 0x01
#define RFM69_REG_DATAMODUL 0x02
#define RFM69_REG_BITRATEMSB 0x03
#define RFM69_REG_BITRATELSB 0x04
#define RFM69_REG_FDEVMSB 0x05
#define RFM69_REG_FDEVLSB 0x06
#define RFM69_REG_FRFMSB 0x07
#define RFM69_REG_FRFMID 0x08
#define RFM69_REG_FRFLSB 0x09
#define RFM69_REG_OSC1 0x0A
#define RFM69_REG_AFCCTRL 0x0B
#define RFM69_REG_LOWBAT 0x0C
#define RFM69_REG_LISTEN1 0x0D
#define RFM69_REG_LISTEN2 0x0E
#define RFM69_REG_LISTEN3 0x0F
#define RFM69_REG_VERSION 0x10
#define RFM69_REG_PALEVEL 0x11
#define RFM69_REG_PARAMP 0x12
#define RFM69_REG_OCP 0x13
#define RFM69_REG_LNA 0x18
#define RFM69_REG_RXBW 0x19
#define RFM69_REG_AFCBW 0x1A
#define RFM69_REG_OOKPEAK 0x1B
#define RFM69_REG_OOKAVG 0x1C
#define RFM69_REG_OOKFIX 0x1D
#define RFM69_REG_AFCFEI 0x1E
#define RFM69_REG_AFCMSB 0x1F
#define RFM69_REG_AFCLSB 0x20
#define RFM69_REG_FEIMSB 0x21
#define RFM69_REG_FEILSB 0x22
#define RFM69_REG_RSSICONFIG 0x23
#define RFM69_REG_RSSIVALUE 0x24
#define RFM69_REG_DIOMAPPING1 0x25
#define RFM69_REG_DIOMAPPING2 0x26
#define RFM69_REG_IRQFLAGS1 0x27
#define RFM69_REG_IRQFLAGS2 0x28
#define RFM69_REG_RSSITHRESH 0x29
#define RFM69_REG_RXTIMEOUT1 0x2A
#define RFM69_REG_RXTIMEOUT2 0x2B
#define RFM69_REG_PREAMBLEMSB 0x2C
#define RFM69_REG_PREAMBLELSB 0x2D
#define RFM69_REG_SYNCCONFIG 0x2E
#define RFM69_REG_SYNCVALUE1 0x2F
#define RFM69_REG_SYNCVALUE2 0x30
#define RFM69_REG_SYNCVALUE3 0x31
#define RFM69_REG_SYNCVALUE4 0x32
#define RFM69_REG_SYNCVALUE5 0x33
#define RFM69_REG_SYNCVALUE6 0x34
#define RFM69_REG_SYNCVALUE7 0x35
#define RFM69_REG_SYNCVALUE8 0x36
#define RFM69_REG_PACKETCONFIG1 0x37
#define RFM69_REG_PAYLOADLENGTH 0x38
#define RFM69_REG_NODEADRS 0x39
#define RFM69_REG_BROADCASTADRS 0x3A
#define RFM69_REG_AUTOMODES 0x3B
#define RFM69_REG_FIFOTHRESH 0x3C
#define RFM69_REG_PACKETCONFIG2 0x3D
#define RFM69_REG_AESKEY1 0x3E
#define RFM69_REG_AESKEY2 0x3F
#define RFM69_REG_AESKEY3 0x40
#define RFM69_REG_AESKEY4 0x41
#define RFM69_REG_AESKEY5 0x42
#define RFM69_REG_AESKEY6 0x43
#define RFM69_REG_AESKEY7 0x44
#define RFM69_REG_AESKEY8 0x45
#define RFM69_REG_AESKEY9 0x46
#define RFM69_REG_AESKEY10 0x47
#define RFM69_REG_AESKEY11 0x48
#define RFM69_REG_AESKEY12 0x49
#define RFM69_REG_AESKEY13 0x4A
#define RFM69_REG_AESKEY14 0x4B
#define RFM69_REG_AESKEY15 0x4C
#define RFM69_REG_AESKEY16 0x4D
#define RFM69_REG_TEMP1 0x4E
#define RFM69_REG_TEMP2 0x4F
#define RFM69_REG_TESTLNA 0x58
#define RFM69_REG_TESTPA1 0x5A
#define RFM69_REG_TESTPA2 0x5C
#define RFM69_REG_TESTDAGC 0x6F

typedef enum {
  RFM69_MODE_SLEEP = 0,
  RFM69_MODE_STANDBY = 1,
  RFM69_MODE_TX = 3,
  RFM69_MODE_RX = 4,
} RFM69_Mode;

void RFM69_init();
void RFM69_write(uint8_t addr, uint8_t val);
void RFM69_write16(uint8_t addr, uint16_t val);
uint8_t RFM69_read(uint8_t addr);
void RFM69_set_mode(RFM69_Mode mode);
