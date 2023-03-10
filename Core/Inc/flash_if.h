#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_flash.h"

void FLASH_If_Init(void);
signed char FLASH_If_Erase(void);
uint32_t FLASH_If_Write(__IO uint32_t *FlashAddress, uint32_t *Data, uint16_t DataLength);
void FLASH_If_UnInit(void);

#endif
