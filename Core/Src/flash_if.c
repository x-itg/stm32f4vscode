// 用于升级程序
/* Device voltage range supposed to be [2.7V to 3.6V], the operation will be done by word */
// STM32F411RET6  FLASH 512Kbytes
// FLASH_SECTOR_0   16K   0x8000000 Bootloader -|
// FLASH_SECTOR_1   16K   0x8004000 Bootloader -|32Kbytes
// FLASH_SECTOR_2   16K   0x8008000 Parameter ----|
// FLASH_SECTOR_3   16K   0x800C000 Parameter ----|32Kbytes
// FLASH_SECTOR_4   64K   0x8010000 Application ----|
// FLASH_SECTOR_5   128K  0x8020000 Application     |
// FLASH_SECTOR_6   128K  0x8040000 Application     |
// FLASH_SECTOR_7   128K  0x8060000 Application ----|448Kbytes

#include "flash_if.h"

#define USER_FLASH_FIRST_PAGE_ADDRESS 0x08010000
#define USER_FLASH_LAST_PAGE_ADDRESS 0x08060000
#define USER_FLASH_END_ADDRESS 0x0807FFFF
// 解锁
void FLASH_If_Init(void)
{
  HAL_FLASH_Unlock(); // 解锁flash控制块
}

// 擦除APP
signed char FLASH_If_Erase(void)
{
  uint32_t FlashAddress;

  FlashAddress = 0x08010000;

  if (FlashAddress <= (uint32_t)USER_FLASH_LAST_PAGE_ADDRESS)
  {
    FLASH_EraseInitTypeDef FLASH_EraseInitStruct;
    uint32_t sectornb = 0;

    FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    FLASH_EraseInitStruct.Sector = FLASH_SECTOR_4; // 0x8010000
    FLASH_EraseInitStruct.NbSectors = 4;           // 448K--4、5、6、7这四个扇区
    FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &sectornb) != HAL_OK)
    {
      return (1);
    }
  }
  else
  {
    return (1);
  }

  return (0);
}

// 写入
uint32_t FLASH_If_Write(__IO uint32_t *FlashAddress, uint32_t *Data, uint16_t DataLength)
{
  uint32_t i = 0;

  for (i = 0; (i < DataLength) && (*FlashAddress <= (USER_FLASH_END_ADDRESS - 4)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
    be done by word */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *FlashAddress, *(uint32_t *)(Data + i)) == HAL_OK)
    {
      /* Check the written value */
      if (*(uint32_t *)*FlashAddress != *(uint32_t *)(Data + i))
      {
        /* Flash content doesn't match SRAM content */
        return (2);
      }
      /* Increment FLASH destination address */
      *FlashAddress += 4;
    }
    else
    {
      /* Error occurred while writing data in Flash memory */
      return (1);
    }
  }

  return (0);
}

// 上锁
void FLASH_If_UnInit(void)
{
  HAL_FLASH_Lock();
}
