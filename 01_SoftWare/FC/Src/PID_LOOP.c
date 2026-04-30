#include "PID_LOOP.h"

#include <stddef.h>
#include "Motor_Control.h"
#include "cmsis_os.h"

/* 固定基础油门，单位是 DShot 油门值 */
#define PID_BASE_THROTTLE        50U

/* 电机最小运行油门；混控结果非 0 但低于该值时，会抬到这里防止电机停转。 */
#define PID_MIN_RUNNING_THROTTLE 48U

/* 电机最大允许油门；调试阶段限制上限，避免 PID 算飞后油门过大。 */
#define PID_MAX_THROTTLE         250U

/* 上电后先持续发送 0 油门的时间，让电调完成 DShot 识别和安全解锁。 */
#define PID_ARM_TIME_MS          3000U

/* PID 控制周期，1 ms 对应 1000 Hz。 */ 
#define PID_FRAME_PERIOD_MS      1U

/* 姿态数据超时时间；超过该时间没有新姿态数据就停电机。 */
#define PID_ATTITUDE_TIMEOUT_MS  200U

/* Roll 角度环 P；角度误差转成目标角速度。先调角速度环时保持 0。 */
#define PID_ANGLE_ROLL_KP        0.0f

/* Pitch 角度环 P；角度误差转成目标角速度。先调角速度环时保持 0。 */
#define PID_ANGLE_PITCH_KP       0.0f

/* 角度环输出限幅，限制目标角速度最大值，单位 deg/s。 */
#define PID_ANGLE_RATE_LIMIT     120.0f

/* Roll 角速度环 P；主要力度参数，太小无力，太大高频震荡。 */
#define PID_RATE_ROLL_KP         0.65f

/* Roll 角速度环 I；消除长期偏差，先调 P 时建议设为 0。 */
#define PID_RATE_ROLL_KI         0.02f

/* Roll 角速度环 D；抑制快速变化，噪声没处理好前建议为 0。 */
#define PID_RATE_ROLL_KD         0.00f

/* Pitch 角速度环 P；主要力度参数，太小无力，太大高频震荡。 */
#define PID_RATE_PITCH_KP        0.35f

/* Pitch 角速度环 I；消除长期偏差，先调 P 时建议设为 0。 */
#define PID_RATE_PITCH_KI        0.02f

/* Pitch 角速度环 D；抑制快速变化，噪声没处理好前建议为 0。 */
#define PID_RATE_PITCH_KD        0.00f

/* Yaw 角速度环 P；抑制偏航角速度，目前 yaw 不做角度回正。 */
#define PID_RATE_YAW_KP          0.20f

/* Yaw 角速度环 I；消除偏航长期偏差，初期建议为 0。 */
#define PID_RATE_YAW_KI          0.00f

/* Yaw 角速度环 D；偏航阻尼，初期建议为 0。 */
#define PID_RATE_YAW_KD          0.00f

/* PID 对单个电机的最大修正量，防止姿态修正一次拉太大。 */
#define PID_MOTOR_CORR_LIMIT     45.0f

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float integral_limit;
  float output_limit;
} PID_Controller_t;

static volatile int16_t s_att_pitch_cd = 0;
static volatile int16_t s_att_roll_cd = 0;
static volatile int16_t s_gyro_x_cdps = 0;
static volatile int16_t s_gyro_y_cdps = 0;
static volatile int16_t s_gyro_z_cdps = 0;
static volatile uint8_t s_attitude_ready = 0U;
static volatile uint32_t s_attitude_tick = 0U;
static volatile uint16_t s_motor1_out = 0U;
static volatile uint16_t s_motor2_out = 0U;
static volatile uint16_t s_motor3_out = 0U;
static volatile uint16_t s_motor4_out = 0U;

static float PID_LimitFloat(float value, float limit);
static void PID_Reset(PID_Controller_t *pid);
static float PID_Update(PID_Controller_t *pid, float error, float dt);
static uint16_t PID_MotorClamp(float value);

void PID_LOOP_UpdateAttitude(int16_t pitch_cd, int16_t roll_cd, int16_t yaw_cd,
                             float gyro_x_dps, float gyro_y_dps, float gyro_z_dps)
{
  (void)yaw_cd;
  s_att_pitch_cd = pitch_cd;
  s_att_roll_cd = roll_cd;
  s_gyro_x_cdps = (int16_t)(gyro_x_dps * 100.0f);
  s_gyro_y_cdps = (int16_t)(gyro_y_dps * 100.0f);
  s_gyro_z_cdps = (int16_t)(gyro_z_dps * 100.0f);
  s_attitude_tick = HAL_GetTick();
  s_attitude_ready = 1U;
}

void PID_LOOP_Run(void)
{
  PID_Controller_t roll_angle_pid = {
    .kp = PID_ANGLE_ROLL_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = PID_ANGLE_RATE_LIMIT,
    .output_limit = PID_ANGLE_RATE_LIMIT
  };
  PID_Controller_t pitch_angle_pid = {
    .kp = PID_ANGLE_PITCH_KP,
    .ki = 0.0f,
    .kd = 0.0f,
    .integral_limit = PID_ANGLE_RATE_LIMIT,
    .output_limit = PID_ANGLE_RATE_LIMIT
  };
  PID_Controller_t roll_rate_pid = {
    .kp = PID_RATE_ROLL_KP,
    .ki = PID_RATE_ROLL_KI,
    .kd = PID_RATE_ROLL_KD,
    .integral_limit = PID_MOTOR_CORR_LIMIT,
    .output_limit = PID_MOTOR_CORR_LIMIT
  };
  PID_Controller_t pitch_rate_pid = {
    .kp = PID_RATE_PITCH_KP,
    .ki = PID_RATE_PITCH_KI,
    .kd = PID_RATE_PITCH_KD,
    .integral_limit = PID_MOTOR_CORR_LIMIT,
    .output_limit = PID_MOTOR_CORR_LIMIT
  };
  PID_Controller_t yaw_rate_pid = {
    .kp = PID_RATE_YAW_KP,
    .ki = PID_RATE_YAW_KI,
    .kd = PID_RATE_YAW_KD,
    .integral_limit = PID_MOTOR_CORR_LIMIT,
    .output_limit = PID_MOTOR_CORR_LIMIT
  };
  uint32_t next_tick = osKernelGetTickCount();
  uint32_t start_tick = HAL_GetTick();
  uint32_t attitude_tick = 0U;
  int16_t pitch_cd = 0;
  int16_t roll_cd = 0;
  int16_t gyro_x_cdps = 0;
  int16_t gyro_y_cdps = 0;
  int16_t gyro_z_cdps = 0;
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float gyro_x_dps = 0.0f;
  float gyro_y_dps = 0.0f;
  float gyro_z_dps = 0.0f;
  float roll_target_rate = 0.0f;
  float pitch_target_rate = 0.0f;
  float roll_correction = 0.0f;
  float pitch_correction = 0.0f;
  float yaw_correction = 0.0f;
  float m1 = 0.0f;
  float m2 = 0.0f;
  float m3 = 0.0f;
  float m4 = 0.0f;
  const float dt = (float)PID_FRAME_PERIOD_MS * 0.001f;

  Motor_Init();

  while ((HAL_GetTick() - start_tick) < PID_ARM_TIME_MS)
  {
    (void)Motor_Write(0U, 0U, 0U, 0U);
    next_tick += PID_FRAME_PERIOD_MS;
    (void)osDelayUntil(next_tick);
  }

  PID_Reset(&roll_angle_pid);
  PID_Reset(&pitch_angle_pid);
  PID_Reset(&roll_rate_pid);
  PID_Reset(&pitch_rate_pid);
  PID_Reset(&yaw_rate_pid);

  for (;;)
  {
    attitude_tick = s_attitude_tick;
    if ((s_attitude_ready == 0U) || ((HAL_GetTick() - attitude_tick) > PID_ATTITUDE_TIMEOUT_MS))
    {
      PID_Reset(&roll_angle_pid);
      PID_Reset(&pitch_angle_pid);
      PID_Reset(&roll_rate_pid);
      PID_Reset(&pitch_rate_pid);
      PID_Reset(&yaw_rate_pid);
      (void)Motor_Write(0U, 0U, 0U, 0U);
    }
    else
    {
      pitch_cd = s_att_pitch_cd;
      roll_cd = s_att_roll_cd;
      gyro_x_cdps = s_gyro_x_cdps;
      gyro_y_cdps = s_gyro_y_cdps;
      gyro_z_cdps = s_gyro_z_cdps;

      pitch_deg = (float)pitch_cd * 0.01f;
      roll_deg = (float)roll_cd * 0.01f;
      gyro_x_dps = (float)gyro_x_cdps * 0.01f;
      gyro_y_dps = (float)gyro_y_cdps * 0.01f;
      gyro_z_dps = (float)gyro_z_cdps * 0.01f;

      roll_target_rate = PID_Update(&roll_angle_pid, -roll_deg, dt);
      pitch_target_rate = PID_Update(&pitch_angle_pid, -pitch_deg, dt);
      roll_correction = PID_Update(&roll_rate_pid, roll_target_rate - gyro_x_dps, dt);
      pitch_correction = PID_Update(&pitch_rate_pid, pitch_target_rate - gyro_y_dps, dt);
      yaw_correction = PID_Update(&yaw_rate_pid, -gyro_z_dps, dt);

      m1 = (float)PID_BASE_THROTTLE - roll_correction + pitch_correction - yaw_correction;
      m2 = (float)PID_BASE_THROTTLE - roll_correction - pitch_correction + yaw_correction;
      m3 = (float)PID_BASE_THROTTLE + roll_correction + pitch_correction + yaw_correction;
      m4 = (float)PID_BASE_THROTTLE + roll_correction - pitch_correction - yaw_correction;

      s_motor1_out = PID_MotorClamp(m1);
      s_motor2_out = PID_MotorClamp(m2);
      s_motor3_out = PID_MotorClamp(m3);
      s_motor4_out = PID_MotorClamp(m4);
      (void)Motor_Write(s_motor1_out, s_motor2_out, s_motor3_out, s_motor4_out);
    }

    next_tick += PID_FRAME_PERIOD_MS;
    (void)osDelayUntil(next_tick);
  }
}

static float PID_LimitFloat(float value, float limit)
{
  if (value > limit)
  {
    return limit;
  }

  if (value < -limit)
  {
    return -limit;
  }

  return value;
}

static void PID_Reset(PID_Controller_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

static float PID_Update(PID_Controller_t *pid, float error, float dt)
{
  float derivative = 0.0f;
  float output = 0.0f;

  if ((pid == NULL) || (dt <= 0.0f))
  {
    return 0.0f;
  }

  pid->integral += error * dt;
  pid->integral = PID_LimitFloat(pid->integral, pid->integral_limit);
  derivative = (error - pid->last_error) / dt;
  pid->last_error = error;

  output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  return PID_LimitFloat(output, pid->output_limit);
}

static uint16_t PID_MotorClamp(float value)
{
  if (value <= 0.0f)
  {
    return 0U;
  }

  if (value < (float)PID_MIN_RUNNING_THROTTLE)
  {
    return PID_MIN_RUNNING_THROTTLE;
  }

  if (value > (float)PID_MAX_THROTTLE)
  {
    return PID_MAX_THROTTLE;
  }

  return (uint16_t)(value + 0.5f);
}
