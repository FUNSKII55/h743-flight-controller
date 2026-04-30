#include "ICM42688P.h"

#include <stddef.h>
#include "main.h"
#include "spi.h"

#define IMU1_WHO_AM_I_REG        0x75U
#define IMU1_ACCEL_DATA_X1_REG   0x1FU
#define IMU1_GYRO_DATA_X1_REG    0x25U
#define IMU1_PWR_MGMT0_REG       0x4EU
#define IMU1_GYRO_CONFIG0_REG    0x4FU
#define IMU1_ACCEL_CONFIG0_REG   0x50U

static void ICM42688P_WriteReg(uint8_t reg, uint8_t value);
static void ICM42688P_ReadBurst(uint8_t reg, uint8_t *rx, uint16_t len);

uint8_t ICM42688P_Init(void)
{
  ICM42688P_WriteReg(IMU1_PWR_MGMT0_REG, 0x0FU);        //Low Noise 模式
  ICM42688P_WriteReg(IMU1_GYRO_CONFIG0_REG, 0x06U);     
  ICM42688P_WriteReg(IMU1_ACCEL_CONFIG0_REG, 0x06U);
  HAL_Delay(100);

  return ICM42688P_ReadWhoAmI();                                    //读验证
}

uint8_t ICM42688P_ReadWhoAmI(void)
{
  uint8_t tx[2] = {IMU1_WHO_AM_I_REG | 0x80U, 0x00U};
  uint8_t rx[2] = {0U, 0U};

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2U, 100U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  return rx[1];
}

void ICM42688P_ReadAccelRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t rx[7] = {0U};

  if ((ax == NULL) || (ay == NULL) || (az == NULL))
  {
    return;
  }

  ICM42688P_ReadBurst(IMU1_ACCEL_DATA_X1_REG, rx, 7U);

  *ax = (int16_t)((rx[1] << 8) | rx[2]);
  *ay = (int16_t)((rx[3] << 8) | rx[4]);
  *az = (int16_t)((rx[5] << 8) | rx[6]);
}

void ICM42688P_ReadGyroRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t rx[7] = {0U};

  if ((gx == NULL) || (gy == NULL) || (gz == NULL))
  {
    return;
  }

  ICM42688P_ReadBurst(IMU1_GYRO_DATA_X1_REG, rx, 7U);

  *gx = (int16_t)((rx[1] << 8) | rx[2]);
  *gy = (int16_t)((rx[3] << 8) | rx[4]);
  *gz = (int16_t)((rx[5] << 8) | rx[6]);
}

static void ICM42688P_WriteReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = {reg & 0x7FU, value};

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
  (void)HAL_SPI_Transmit(&hspi1, tx, 2U, 100U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}

static void ICM42688P_ReadBurst(uint8_t reg, uint8_t *rx, uint16_t len)
{
  uint8_t tx[7] = {0U};
  uint16_t i = 0U;

  if ((rx == NULL) || (len > sizeof(tx)))
  {
    return;
  }

  tx[0] = reg | 0x80U;
  for (i = 1U; i < len; i++)
  {
    tx[i] = 0U;
  }

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
  (void)HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, 100U);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
}
