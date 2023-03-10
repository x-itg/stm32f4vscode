#ifndef _EEPROM_H_
#define _EEPROM_H_
#include "main.h"
#include "stdint.h"
#include "string.h"
#include "stdio.h"
// 参数结构体
typedef struct
{
   uint32_t boot_count;    // 0
   uint16_t modbus_addr;   // 4
   int32_t Voltage_offset; // 6
   int32_t Current_offset; // 10
   uint16_t boof;
} Config_type;
extern uint32_t boot_count;
extern uint32_t Voltage;
extern uint16_t bootloaderflag;
/* eeprom storage data num max, 1-65535, unit halfword.
Must be smaller than (EEPROM_PART_SIZE/4)-2 */
#define EEPROM_NUM_MAX 8 //*2byte

/* EEPROM Use two partitions, each partition size is
an integer multiple of the erased page */
#define EEPROM_PART0_SIZE (0x4000) // eeprom part size 16K, total size 32K
#define EEPROM_PART1_SIZE (0x4000)

/* EEPROM start address in Flash */
#define EEPROM_START_ADDRESS (0x08008000U) // use flash 0x08008000 - 0x08010000, for 512k flash mcu
#define EEPROM_END_ADDRESS (EEPROM_START_ADDRESS + EEPROM_PART0_SIZE + EEPROM_PART1_SIZE)

/* Pages 0 and 1 base and end addresses */
#define PART0_BASE_ADDRESS (EEPROM_START_ADDRESS)
#define PART0_END_ADDRESS ((PART0_BASE_ADDRESS + EEPROM_PART0_SIZE))

#define PART1_BASE_ADDRESS (PART0_END_ADDRESS)
#define PART1_END_ADDRESS ((PART1_BASE_ADDRESS + EEPROM_PART1_SIZE))

/* PAGE is marked to record data */
#define PART_USED_MARK (0xEAE5D135 + EEPROM_NUM_MAX) // Different number, use new data
// #define PART_USED_MARK          (0xEAE5D135)  //Different number, use old data

#ifndef USE_HAL_DRIVER
#define HAL_GetTick() 0
#define HAL_Delay(ms)
typedef enum
{
   HAL_OK = 0x00U,
   HAL_ERROR = 0x01U,
   HAL_BUSY = 0x02U,
   HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;
#endif

int EEPROM_Init(void *default_data);
int EEPROM_Format(void *default_data);
uint16_t EEPROM_Read(uint16_t Address);
int EEPROM_Write(uint16_t Address, uint16_t Data);
int EEPROM_Read_Buf(uint16_t Address, uint16_t *buf, uint16_t length);
int EEPROM_Write_Buf(uint16_t Address, uint16_t *buf, uint16_t length);
#define Config_Read_Buf(addr, buf, length) EEPROM_Read_Buf((addr) / 2, (uint16_t *)(buf), (length) / 2)
#define Config_Write_Buf(addr, buf, length) EEPROM_Write_Buf((addr) / 2, (uint16_t *)(buf), (length) / 2)

/* flash read program erase callback function */
int EE_ErasePart(int part);
int EE_ProgramWord(uint32_t Address, uint32_t Data);
#define EE_ReadWord(Addr) (*(volatile uint32_t *)Addr)

#endif