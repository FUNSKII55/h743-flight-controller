#include "Kalman.h"

#include <math.h>
#include <stddef.h>
#include "cmsis_os.h"
#include "ICM42688P.h"
#include "IST8310.h"

#define IMU1_GYRO_SENS_2000DPS   16.4f
#define RAD_TO_DEG               57.2957795f
#define DEG_TO_RAD               0.0174532925f
#define YAW_GYRO_DEADBAND_DPS    0.30f
#define YAW_MAG_BLEND            0.25f
#define GYRO_BIAS_DISCARD_COUNT  50U

typedef struct
{
  float angle;
  float bias;
  float rate;
  float p00;
  float p01;
  float p10;
  float p11;
  float q_angle;
  float q_bias;
  float r_measure;
} Kalman1D_t;

static Kalman1D_t s_pitch_kalman;
static Kalman1D_t s_roll_kalman;
static Kalman1D_t s_yaw_kalman;
static uint32_t s_last_tick = 0U;

static float s_mag_offset_x = 0.0f;
static float s_mag_offset_y = 0.0f;
static float s_mag_offset_z = 0.0f;
static float s_mag_scale_x = 1.0f;
static float s_mag_scale_y = 1.0f;
static float s_mag_scale_z = 1.0f;
static float s_gyro_bias_x = 0.0f;
static float s_gyro_bias_y = 0.0f;
static float s_gyro_bias_z = 0.0f;
static float s_latest_gyro_x_dps = 0.0f;
static float s_latest_gyro_y_dps = 0.0f;
static float s_latest_gyro_z_dps = 0.0f;
static float s_pitch_zero = 0.0f;
static float s_roll_zero = 0.0f;
static float s_yaw_zero = 0.0f;
static uint8_t s_yaw_zero_valid = 0U;

static void Kalman1D_Reset(Kalman1D_t *kalman);
static float Kalman_NormalizeAngle180(float angle);
static float Kalman_BlendAngle180(float current, float target, float alpha);
static float Kalman1D_Predict(Kalman1D_t *kalman, float new_rate, float dt);
static float Kalman1D_Update(Kalman1D_t *kalman, float new_angle, float new_rate, float dt);
static void Kalman_DelayMs(uint32_t delay_ms);

void Kalman_Init(void)
{
  Kalman1D_Reset(&s_pitch_kalman);
  Kalman1D_Reset(&s_roll_kalman);
  Kalman1D_Reset(&s_yaw_kalman);
  s_pitch_zero = 0.0f;
  s_roll_zero = 0.0f;
  s_yaw_zero = 0.0f;
  s_yaw_zero_valid = 0U;
  s_gyro_bias_x = 0.0f;
  s_gyro_bias_y = 0.0f;
  s_gyro_bias_z = 0.0f;
  s_latest_gyro_x_dps = 0.0f;
  s_latest_gyro_y_dps = 0.0f;
  s_latest_gyro_z_dps = 0.0f;

  s_yaw_kalman.q_angle = 0.05f;
  s_yaw_kalman.q_bias = 0.001f;
  s_yaw_kalman.r_measure = 0.8f;

  s_last_tick = HAL_GetTick();
}

void Kalman_CalibrateGyroBias(uint16_t sample_count, uint32_t sample_delay_ms)
{
  int64_t sum_x = 0;
  int64_t sum_y = 0;
  int64_t sum_z = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;

  if (sample_count == 0U)
  {
    return;
  }

  for (uint16_t i = 0U; i < GYRO_BIAS_DISCARD_COUNT; i++)
  {
    ICM42688P_ReadGyroRaw(&gx, &gy, &gz);
    Kalman_DelayMs(sample_delay_ms);
  }

  /* 上电静止采样陀螺仪零偏，避免 yaw 开机先漂再被磁力计拉回。 */
  for (uint16_t i = 0U; i < sample_count; i++)
  {
    ICM42688P_ReadGyroRaw(&gx, &gy, &gz);
    sum_x += gx;
    sum_y += gy;
    sum_z += gz;

    if (sample_delay_ms > 0U)
    {
      Kalman_DelayMs(sample_delay_ms);
    }
  }

  s_gyro_bias_x = ((float)sum_x / (float)sample_count) / IMU1_GYRO_SENS_2000DPS;
  s_gyro_bias_y = ((float)sum_y / (float)sample_count) / IMU1_GYRO_SENS_2000DPS;
  s_gyro_bias_z = ((float)sum_z / (float)sample_count) / IMU1_GYRO_SENS_2000DPS;
  s_last_tick = HAL_GetTick();
}

void Kalman_ResetReference(void)
{
  s_pitch_zero = s_pitch_kalman.angle;
  s_roll_zero = s_roll_kalman.angle;

  if (s_yaw_zero_valid != 0U)
  {
    s_yaw_zero = s_yaw_kalman.angle;
  }

  s_last_tick = HAL_GetTick();
}

void Kalman_SetMagCalibration(float offset_x, float offset_y, float offset_z,
                              float scale_x, float scale_y, float scale_z)
{
  s_mag_offset_x = offset_x;
  s_mag_offset_y = offset_y;
  s_mag_offset_z = offset_z;
  s_mag_scale_x = scale_x;
  s_mag_scale_y = scale_y;
  s_mag_scale_z = scale_z;
}

void Kalman_GetGyroDps(float *gx_dps, float *gy_dps, float *gz_dps)
{
  if (gx_dps != NULL)
  {
    *gx_dps = s_latest_gyro_x_dps;
  }

  if (gy_dps != NULL)
  {
    *gy_dps = s_latest_gyro_y_dps;
  }

  if (gz_dps != NULL)
  {
    *gz_dps = s_latest_gyro_z_dps;
  }
}

uint8_t Kalman_GetAttitude(int16_t *pitch_cdeg, int16_t *roll_cdeg, int16_t *yaw_cdeg)
{
  uint32_t now_tick = 0U;
  float dt = 0.02f;
  float pitch_acc = 0.0f;
  float roll_acc = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;
  float yaw = 0.0f;
  float pitch_rad = 0.0f;
  float roll_rad = 0.0f;
  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  float mag_x = 0.0f;
  float mag_y = 0.0f;
  float mag_z = 0.0f;
  float mag_x_comp = 0.0f;
  float mag_y_comp = 0.0f;
  float yaw_mag = 0.0f;
  uint8_t mag_valid = 0U;
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  int16_t mx = 0;
  int16_t my = 0;
  int16_t mz = 0;

  if ((pitch_cdeg == NULL) || (roll_cdeg == NULL) || (yaw_cdeg == NULL))
  {
    return 0U;
  }

  now_tick = HAL_GetTick();
  dt = (float)(now_tick - s_last_tick) * 0.001f;
  if (dt <= 0.0f)
  {
    dt = 0.001f;
  }
  s_last_tick = now_tick;

  ICM42688P_ReadAccelRaw(&ax, &ay, &az);
  ICM42688P_ReadGyroRaw(&gx, &gy, &gz);

  gyro_x_dps = ((float)gx / IMU1_GYRO_SENS_2000DPS) - s_gyro_bias_x;
  gyro_y_dps = ((float)gy / IMU1_GYRO_SENS_2000DPS) - s_gyro_bias_y;
  gyro_z_dps = ((float)gz / IMU1_GYRO_SENS_2000DPS) - s_gyro_bias_z;
  if (fabsf(gyro_z_dps) < YAW_GYRO_DEADBAND_DPS)
  {
    gyro_z_dps = 0.0f;
  }
  s_latest_gyro_x_dps = gyro_x_dps;
  s_latest_gyro_y_dps = gyro_y_dps;
  s_latest_gyro_z_dps = gyro_z_dps;

  roll_acc = atan2f((float)ay, (float)az) * RAD_TO_DEG;
  pitch_acc = atan2f(-(float)ax, sqrtf((float)ay * (float)ay + (float)az * (float)az)) * RAD_TO_DEG;

  roll = Kalman1D_Update(&s_roll_kalman, roll_acc, gyro_x_dps, dt);
  pitch = Kalman1D_Update(&s_pitch_kalman, pitch_acc, gyro_y_dps, dt);

  if (IST8310_ReadMagRaw(&mx, &my, &mz) != 0U)
  {
    pitch_rad = pitch * DEG_TO_RAD;
    roll_rad = roll * DEG_TO_RAD;
    mag_x = ((float)mx - s_mag_offset_x) * s_mag_scale_x;
    mag_y = ((float)my - s_mag_offset_y) * s_mag_scale_y;
    mag_z = ((float)mz - s_mag_offset_z) * s_mag_scale_z;

    mag_x_comp = mag_x * cosf(pitch_rad) + mag_z * sinf(pitch_rad);
    mag_y_comp = mag_x * sinf(roll_rad) * sinf(pitch_rad)
               + mag_y * cosf(roll_rad)
               - mag_z * sinf(roll_rad) * cosf(pitch_rad);

    yaw_mag = Kalman_NormalizeAngle180(atan2f(-mag_y_comp, mag_x_comp) * RAD_TO_DEG);
    mag_valid = 1U;
    if (s_yaw_zero_valid == 0U)
    {
      s_yaw_kalman.angle = yaw_mag;
      s_yaw_kalman.bias = 0.0f;
      s_yaw_zero = yaw_mag;
      s_yaw_zero_valid = 1U;
      yaw = yaw_mag;
    }
    else
    {
      yaw = Kalman1D_Predict(&s_yaw_kalman, gyro_z_dps, dt);
      yaw = Kalman_BlendAngle180(yaw, yaw_mag, YAW_MAG_BLEND);
      s_yaw_kalman.angle = yaw;
    }
  }
  else
  {
    yaw = Kalman1D_Predict(&s_yaw_kalman, gyro_z_dps, dt);
  }

  *pitch_cdeg = (int16_t)((pitch - s_pitch_zero) * 100.0f);
  *roll_cdeg = (int16_t)((roll - s_roll_zero) * 100.0f);
  yaw = Kalman_NormalizeAngle180(yaw);
  if ((s_yaw_zero_valid == 0U) && (mag_valid == 0U))
  {
    *yaw_cdeg = 0;
  }
  else
  {
    *yaw_cdeg = (int16_t)(Kalman_NormalizeAngle180(yaw - s_yaw_zero) * 100.0f);
  }

  return ICM42688P_ReadWhoAmI();
}

static void Kalman1D_Reset(Kalman1D_t *kalman)
{
  kalman->angle = 0.0f;
  kalman->bias = 0.0f;
  kalman->rate = 0.0f;
  kalman->p00 = 1.0f;
  kalman->p01 = 0.0f;
  kalman->p10 = 0.0f;
  kalman->p11 = 1.0f;
  kalman->q_angle = 0.001f;
  kalman->q_bias = 0.003f;
  kalman->r_measure = 0.03f;
}

static float Kalman_NormalizeAngle180(float angle)
{
  while (angle > 180.0f)
  {
    angle -= 360.0f;
  }

  while (angle < -180.0f)
  {
    angle += 360.0f;
  }

  return angle;
}

static float Kalman_BlendAngle180(float current, float target, float alpha)
{
  float error = Kalman_NormalizeAngle180(target - current);
  return Kalman_NormalizeAngle180(current + (error * alpha));
}

static float Kalman1D_Predict(Kalman1D_t *kalman, float new_rate, float dt)
{
  kalman->rate = new_rate - kalman->bias;
  kalman->angle += dt * kalman->rate;
  kalman->angle = Kalman_NormalizeAngle180(kalman->angle);

  kalman->p00 += dt * (dt * kalman->p11 - kalman->p01 - kalman->p10 + kalman->q_angle);
  kalman->p01 -= dt * kalman->p11;
  kalman->p10 -= dt * kalman->p11;
  kalman->p11 += kalman->q_bias * dt;

  return kalman->angle;
}

static float Kalman1D_Update(Kalman1D_t *kalman, float new_angle, float new_rate, float dt)
{
  float s = 0.0f;
  float k0 = 0.0f;
  float k1 = 0.0f;
  float y = 0.0f;
  float p00_temp = 0.0f;
  float p01_temp = 0.0f;

  kalman->rate = new_rate - kalman->bias;
  kalman->angle += dt * kalman->rate;

  kalman->p00 += dt * (dt * kalman->p11 - kalman->p01 - kalman->p10 + kalman->q_angle);
  kalman->p01 -= dt * kalman->p11;
  kalman->p10 -= dt * kalman->p11;
  kalman->p11 += kalman->q_bias * dt;

  s = kalman->p00 + kalman->r_measure;
  k0 = kalman->p00 / s;
  k1 = kalman->p10 / s;
  y = new_angle - kalman->angle;

  kalman->angle += k0 * y;
  kalman->bias += k1 * y;

  p00_temp = kalman->p00;
  p01_temp = kalman->p01;

  kalman->p00 -= k0 * p00_temp;
  kalman->p01 -= k0 * p01_temp;
  kalman->p10 -= k1 * p00_temp;
  kalman->p11 -= k1 * p01_temp;

  return kalman->angle;
}

static void Kalman_DelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0U)
  {
    return;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(delay_ms);
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}
