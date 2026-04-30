#include "IST8310.h"

#include <stddef.h>
#include "i2c.h"

#define IST8310_I2C_ADDR_7BIT      0x0EU
#define IST8310_I2C_ADDR           (IST8310_I2C_ADDR_7BIT << 1)

#define IST8310_WHO_AM_I_REG       0x00U
#define IST8310_DATA_XL_REG        0x03U
#define IST8310_CNTL1_REG          0x0AU
#define IST8310_AVGCNTL_REG        0x41U
#define IST8310_PDCNTL_REG         0x42U

#define IST8310_SINGLE_MEASURE     0x01U

uint8_t IST8310_Init(void)
{
  uint8_t pd_value = 0xC0U;
  uint8_t avg_value = 0x24U;

  (void)HAL_I2C_Mem_Write(&hi2c2, IST8310_I2C_ADDR, IST8310_PDCNTL_REG,
                          I2C_MEMADD_SIZE_8BIT, &pd_value, 1U, 100U);
  (void)HAL_I2C_Mem_Write(&hi2c2, IST8310_I2C_ADDR, IST8310_AVGCNTL_REG,
                          I2C_MEMADD_SIZE_8BIT, &avg_value, 1U, 100U);

  return IST8310_ReadWhoAmI();
}

uint8_t IST8310_ReadWhoAmI(void)
{
  uint8_t who_am_i = 0U;

  (void)HAL_I2C_Mem_Read(&hi2c2, IST8310_I2C_ADDR, IST8310_WHO_AM_I_REG,
                         I2C_MEMADD_SIZE_8BIT, &who_am_i, 1U, 100U);

  return who_am_i;
}

uint8_t IST8310_ReadMagRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
  uint8_t start = IST8310_SINGLE_MEASURE;
  uint8_t rx[6] = {0U};

  if ((mx == NULL) || (my == NULL) || (mz == NULL))
  {
    return 0U;
  }

  if (HAL_I2C_Mem_Write(&hi2c2, IST8310_I2C_ADDR, IST8310_CNTL1_REG,
                        I2C_MEMADD_SIZE_8BIT, &start, 1U, 100U) != HAL_OK)
  {
    return 0U;
  }

  HAL_Delay(6);

  if (HAL_I2C_Mem_Read(&hi2c2, IST8310_I2C_ADDR, IST8310_DATA_XL_REG,
                       I2C_MEMADD_SIZE_8BIT, rx, sizeof(rx), 100U) != HAL_OK)
  {
    return 0U;
  }

  *mx = (int16_t)((rx[1] << 8) | rx[0]);
  *my = (int16_t)((rx[3] << 8) | rx[2]);
  *mz = (int16_t)((rx[5] << 8) | rx[4]);

  return 1U;
}
