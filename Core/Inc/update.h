#ifndef _UPDATE_H_
#define _UPDATE_H_
#include "main.h"
#include "eeprom.h"
#include "flash_if.h"

void exPool(void);

extern unsigned char rsRxBuf[MAXU2MAX];
extern unsigned int rsRxLen;
extern unsigned int rsRxTime;
extern unsigned int rsRxIndex;
extern unsigned char rsPackFlag;
extern unsigned short part_num;
extern unsigned short part_index;
extern unsigned short part_curIndexLen;
extern unsigned char FlashLockflag;

#endif