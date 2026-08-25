#include "calibration_store.h"

#include <string.h>

/* Reserve the final 2-Kbyte page of the STM32G474's 512-Kbyte internal Flash. */
#define CALIBRATION_FLASH_ADDRESS 0x0807F800UL
#define CALIBRATION_MAGIC 0x434F4E42UL /* "BNOC" in little-endian */
#define CALIBRATION_VERSION 1U
#define CALIBRATION_RECORD_SIZE 40U

static uint32_t Checksum(const uint8_t *data, uint32_t length)
{
  uint32_t sum = 0x13572468UL;

  for (uint32_t index = 0U; index < length; index++)
  {
    sum = (sum << 5U) | (sum >> 27U);
    sum += data[index];
  }
  return sum;
}

static void PutU16LE(uint8_t *destination, uint16_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
}

static void PutU32LE(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)value;
  destination[1] = (uint8_t)(value >> 8U);
  destination[2] = (uint8_t)(value >> 16U);
  destination[3] = (uint8_t)(value >> 24U);
}

static uint16_t GetU16LE(const uint8_t *source)
{
  return (uint16_t)source[0] | ((uint16_t)source[1] << 8U);
}

static uint32_t GetU32LE(const uint8_t *source)
{
  return (uint32_t)source[0] |
         ((uint32_t)source[1] << 8U) |
         ((uint32_t)source[2] << 16U) |
         ((uint32_t)source[3] << 24U);
}

static void ReadRecord(uint8_t record[CALIBRATION_RECORD_SIZE])
{
  memcpy(record, (const void *)CALIBRATION_FLASH_ADDRESS,
         CALIBRATION_RECORD_SIZE);
}

static void GetEraseLocation(uint32_t *bank, uint32_t *page)
{
#if defined(FLASH_OPTR_DBANK)
  if (READ_BIT(FLASH->OPTR, FLASH_OPTR_DBANK) != 0U)
  {
    *bank = FLASH_BANK_2;
    *page = (CALIBRATION_FLASH_ADDRESS - (FLASH_BASE + FLASH_BANK_SIZE)) /
            FLASH_PAGE_SIZE;
    return;
  }
#endif
  *bank = FLASH_BANK_1;
  *page = (CALIBRATION_FLASH_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
}

static uint8_t WriteRecord(const uint8_t record[CALIBRATION_RECORD_SIZE])
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0xFFFFFFFFUL;
  HAL_StatusTypeDef status;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.NbPages = 1U;
  GetEraseLocation(&erase.Banks, &erase.Page);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return 0U;
  }
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_ALL_ERRORS);
  status = HAL_FLASHEx_Erase(&erase, &page_error);
  if (status == HAL_OK)
  {
    for (uint32_t offset = 0U; offset < CALIBRATION_RECORD_SIZE; offset += 8U)
    {
      uint64_t double_word;
      memcpy(&double_word, &record[offset], sizeof(double_word));
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                 CALIBRATION_FLASH_ADDRESS + offset,
                                 double_word);
      if (status != HAL_OK)
      {
        break;
      }
    }
  }
  (void)HAL_FLASH_Lock();
  return (status == HAL_OK) ? 1U : 0U;
}

void CalibrationStore_Init(void)
{
}

uint8_t CalibrationStore_IsPresent(void)
{
  return 1U;
}

uint32_t CalibrationStore_GetAddress(void)
{
  return CALIBRATION_FLASH_ADDRESS;
}

uint8_t CalibrationStore_Load(BNO055CalibrationProfile *profile)
{
  uint8_t record[CALIBRATION_RECORD_SIZE];

  if (profile == NULL)
  {
    return 0U;
  }
  ReadRecord(record);
  if ((GetU32LE(&record[0]) != CALIBRATION_MAGIC) ||
      (GetU16LE(&record[4]) != CALIBRATION_VERSION) ||
      (GetU16LE(&record[6]) != BNO055_CALIBRATION_PROFILE_SIZE) ||
      (record[8] != 0xA0U) ||
      (GetU32LE(&record[36]) != Checksum(record, 36U)))
  {
    return 0U;
  }

  profile->calibration_status = record[9];
  memcpy(profile->data, &record[12], BNO055_CALIBRATION_PROFILE_SIZE);
  return 1U;
}

uint8_t CalibrationStore_Save(const BNO055CalibrationProfile *profile)
{
  uint8_t record[CALIBRATION_RECORD_SIZE];
  uint8_t verify[CALIBRATION_RECORD_SIZE];

  if (profile == NULL)
  {
    return 0U;
  }
  memset(record, 0xFF, sizeof(record));
  PutU32LE(&record[0], CALIBRATION_MAGIC);
  PutU16LE(&record[4], CALIBRATION_VERSION);
  PutU16LE(&record[6], BNO055_CALIBRATION_PROFILE_SIZE);
  record[8] = 0xA0U;
  record[9] = profile->calibration_status;
  memcpy(&record[12], profile->data, BNO055_CALIBRATION_PROFILE_SIZE);
  PutU32LE(&record[36], Checksum(record, 36U));

  if (WriteRecord(record) == 0U)
  {
    return 0U;
  }
  ReadRecord(verify);
  return (memcmp(record, verify, sizeof(record)) == 0) ? 1U : 0U;
}

uint8_t CalibrationStore_Erase(void)
{
  uint8_t empty[CALIBRATION_RECORD_SIZE];

  memset(empty, 0xFF, sizeof(empty));
  return WriteRecord(empty);
}
