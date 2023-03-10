#include "eeprom.h"
static uint16_t EEPROM_data[EEPROM_NUM_MAX] = {0};

static int EEPROM_PART_USE = -1;

static int EE_Init(void);
static int EE_ReadVariable(uint32_t VirtAddress, uint16_t *Data);
static int EE_WriteVariable(uint32_t VirtAddress, uint16_t Data);
static int EE_Format(int part);

#define PAGE_SIZE 0x4000 // 16K

uint32_t boot_count = 0;
uint32_t Voltage;
uint16_t bootloaderflag;

// EEPROM_PART0 0x08008000 - 0x0800BFFF size 16KB
// EEPROM_PART1 0x0800C000 - 0x0800EFFF size 16KB

/* Erase from PART_BASE_ADDRESS, size EEPROM_PART_SIZE */
// 0 0x08008000
// 1 0x0800C000
int EE_ErasePart(int part)
{
  int ret;
  uint32_t erase_address;
  if (part == 0)
  {
    erase_address = PART0_BASE_ADDRESS;
  }
  else
  {
    erase_address = PART1_BASE_ADDRESS;
  }
  if (erase_address < PART0_BASE_ADDRESS || erase_address >= PART1_END_ADDRESS) // 超出范围
    return HAL_ERROR;

  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef FLASH_EraseInitStruct;
  uint32_t sectornb = 0;

  FLASH_EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS; //
  if (part == 0)
    FLASH_EraseInitStruct.Sector = FLASH_SECTOR_2;
  else
    FLASH_EraseInitStruct.Sector = FLASH_SECTOR_3;
  FLASH_EraseInitStruct.NbSectors = 1;
  FLASH_EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  ret = HAL_FLASHEx_Erase(&FLASH_EraseInitStruct, &sectornb);
  if (ret != HAL_OK)
  {
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

/* If the write operation is not a 32-bit atomic operation,
   Write the low 16-bit first, then write the high 16-bit */
int EE_ProgramWord(uint32_t Address, uint32_t Data)
{
  int ret;

  if (Address < PART0_BASE_ADDRESS || Address >= PART1_END_ADDRESS)
    return HAL_ERROR;

  HAL_FLASH_Unlock();

  ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, Data);
  if (ret != HAL_OK)
  {
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }
  ret = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address + 2, Data >> 16);
  if (ret != HAL_OK)
  {
    HAL_FLASH_Lock();
    return HAL_ERROR;
  }

  HAL_FLASH_Lock();
  return HAL_OK;
}

int EEPROM_Init(void *default_data)
{

  if (default_data)
    memcpy(EEPROM_data, default_data, sizeof(EEPROM_data));

  if (EE_Init() != HAL_OK)
  {
    return HAL_ERROR;
  }

  for (int i = 0; i < EEPROM_NUM_MAX; i++)
    EE_ReadVariable(i, &EEPROM_data[i]);

  return HAL_OK;
}

uint16_t EEPROM_Read(uint16_t Address)
{
  if (Address >= EEPROM_NUM_MAX)
    return 0;

  return EEPROM_data[Address];
}

int EEPROM_Write(uint16_t Address, uint16_t Data)
{
  if (EEPROM_PART_USE == -1 || Address >= EEPROM_NUM_MAX)
    return HAL_ERROR;

  if (EEPROM_data[Address] == Data)
    return HAL_OK;

  if (EE_WriteVariable(Address, Data) != HAL_OK)
  {
    // EEPROM_data[Address] = Data;
    return HAL_ERROR;
  }

  return HAL_OK;
}

int EEPROM_Read_Buf(uint16_t Address, uint16_t *buf, uint16_t length)
{
  if (Address + length > EEPROM_NUM_MAX)
    return HAL_ERROR;

  memcpy(buf, EEPROM_data + Address, length << 1);
  return HAL_OK;
}

int EEPROM_Write_Buf(uint16_t Address, uint16_t *buf, uint16_t length)
{
  if (EEPROM_PART_USE == -1 || Address + length > EEPROM_NUM_MAX)
    return HAL_ERROR;

  while (length--)
  {
    if (EEPROM_data[Address] != *buf)
    {
      if (EE_WriteVariable(Address, *buf) != HAL_OK)
      {
        // EEPROM_data[Address] = *buf;
        return HAL_ERROR;
      }
    }

    buf++;
    Address++;
  }
  return HAL_OK;
}

int EEPROM_Format(void *default_data)
{
  if (default_data)
    memcpy(EEPROM_data, default_data, sizeof(EEPROM_data));
  else
    memset(EEPROM_data, 0, sizeof(EEPROM_data));

  if (EE_Format(0) != HAL_OK)
  {
    return HAL_ERROR;
  }

  EEPROM_PART_USE = 0;

  if (EE_ErasePart(1) != HAL_OK)
    return HAL_ERROR;
  return HAL_OK;
}

/* -------------- EE Private function -------------- */
static int EE_Init(void)
{
  if (EE_ReadWord(PART0_BASE_ADDRESS) == PART_USED_MARK)
  {
    EEPROM_PART_USE = 0;
  }
  else if (EE_ReadWord(PART1_BASE_ADDRESS) == PART_USED_MARK)
  {
    EEPROM_PART_USE = 1;
  }
  else
  {
    if (EE_Format(0) != HAL_OK)
      return HAL_ERROR;
    EEPROM_PART_USE = 0;
  }

  return HAL_OK;
}

static int EE_ReadVariable(uint32_t VirtAddress, uint16_t *Data)
{
  uint32_t PartStartAddress = (EEPROM_PART_USE == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
  uint32_t Address = (EEPROM_PART_USE == 0 ? PART0_END_ADDRESS : PART1_END_ADDRESS) - 4;
  uint32_t temp;

  VirtAddress <<= 16;
  for (; Address > PartStartAddress; Address -= 4)
  {
    temp = EE_ReadWord(Address);
    if ((temp & 0xffff0000) == VirtAddress)
    {
      *Data = temp & 0xffff;
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}

static int EE_WriteVariable(uint32_t VirtAddress, uint16_t Data)
{
  uint32_t PartStartAddress = (EEPROM_PART_USE == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
  uint32_t Address = (EEPROM_PART_USE == 0 ? PART0_END_ADDRESS : PART1_END_ADDRESS) - 4;

  if (EE_ReadWord(Address) != 0xffffffff)
  {
    EEPROM_data[VirtAddress] = Data;
    if (EE_Format(EEPROM_PART_USE == 0 ? 1 : 0) != HAL_OK)
      return HAL_ERROR;

    EEPROM_PART_USE = EEPROM_PART_USE == 0 ? 1 : 0;

#if PART0_BASE_ADDRESS != PART1_BASE_ADDRESS
    if (EE_ErasePart(EEPROM_PART_USE == 0 ? 1 : 0) != HAL_OK)
      return HAL_ERROR;
#endif

    return HAL_OK;
  }
  else
  {
    for (Address -= 4; Address > PartStartAddress; Address -= 4)
    {
      if (EE_ReadWord(Address) != 0xffffffff)
        break;
    }

    if (EE_ProgramWord(Address + 4, (VirtAddress << 16) | Data) != HAL_OK)
      return HAL_ERROR;

    EEPROM_data[VirtAddress] = Data;

    return HAL_OK;
  }
}

static int EE_Format(int part)
{
  uint32_t i;
  uint32_t Part_Addr = (part == 0 ? PART0_BASE_ADDRESS : PART1_BASE_ADDRESS);
  uint32_t Address;

  if (EE_ErasePart(part) != HAL_OK)
    return HAL_ERROR;

  Address = Part_Addr + 4;
  for (i = 0; i < EEPROM_NUM_MAX; i++)
  {
    if (EE_ProgramWord(Address, (i << 16) | EEPROM_data[i]) != HAL_OK)
      return HAL_ERROR;

    Address += 4;
  }

  if (EE_ProgramWord(Part_Addr, PART_USED_MARK) != HAL_OK)
    return HAL_ERROR;

  return HAL_OK;
}