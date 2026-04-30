#include "dfu.h"

#include "cmsis_os.h"
#include "uart.h"
#include "usbd_core.h"

#define DFU_BOOTLOADER_ADDR 0x1FF09800UL
#define DFU_CMD_BUF_LEN     8U
#define DFU_BOOT_MAGIC      0x46554E44UL

typedef void (*DFU_JumpFunc)(void);

extern USBD_HandleTypeDef hUsbDeviceFS;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

static char s_dfu_cmd[DFU_CMD_BUF_LEN];
static uint8_t s_dfu_cmd_len = 0U;

static char DFU_ToLower(char c)
{
  if ((c >= 'A') && (c <= 'Z'))
  {
    return (char)(c + ('a' - 'A'));
  }

  return c;
}

static uint8_t DFU_IsSeparator(char c)
{
  return ((c == '\r') || (c == '\n') || (c == ' ') || (c == '\t')) ? 1U : 0U;
}

static void DFU_DelayMs(uint32_t ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    osDelay(ms);
  }
  else
  {
    HAL_Delay(ms);
  }
}

static uint8_t DFU_CommandIsDfu(void)
{
  return ((s_dfu_cmd_len == 3U) &&
          (s_dfu_cmd[0] == 'd') &&
          (s_dfu_cmd[1] == 'f') &&
          (s_dfu_cmd[2] == 'u')) ? 1U : 0U;
}

static void DFU_ResetCommand(void)
{
  s_dfu_cmd_len = 0U;
}

static void DFU_EnableBackupRegister(void)
{
  SET_BIT(PWR->CR1, PWR_CR1_DBP);
  __HAL_RCC_RTC_ENABLE();
}

static void DFU_SetBootMagic(void)
{
  DFU_EnableBackupRegister();
  RTC->BKP0R = DFU_BOOT_MAGIC;
  __DSB();
}

static uint8_t DFU_BootMagicIsSet(void)
{
  DFU_EnableBackupRegister();
  return (RTC->BKP0R == DFU_BOOT_MAGIC) ? 1U : 0U;
}

static void DFU_ClearBootMagic(void)
{
  DFU_EnableBackupRegister();
  RTC->BKP0R = 0U;
  __DSB();
}

static void DFU_HandleCommand(void)
{
  if (DFU_CommandIsDfu() == 0U)
  {
    DFU_ResetCommand();
    return;
  }

  for (uint8_t i = 0U; i < 10U; i++)
  {
    if (UART_SendText("enter dfu\r\n") != 0U)
    {
      break;
    }

    DFU_DelayMs(10U);
  }

  DFU_DelayMs(100U);
  DFU_EnterBootloader();
}

static void DFU_JumpNow(void)
{
  uint32_t boot_stack = *(__IO uint32_t *)DFU_BOOTLOADER_ADDR;
  uint32_t boot_entry = *(__IO uint32_t *)(DFU_BOOTLOADER_ADDR + 4U);
  DFU_JumpFunc boot_jump = (DFU_JumpFunc)boot_entry;

  __disable_irq();

  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;

  for (uint32_t i = 0U; i < 8U; i++)
  {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }

#if (__DCACHE_PRESENT == 1U)
  SCB_CleanInvalidateDCache();
  SCB_DisableDCache();
#endif

#if (__ICACHE_PRESENT == 1U)
  SCB_DisableICache();
#endif

  HAL_MPU_Disable();

  __DSB();
  __ISB();

  SCB->VTOR = DFU_BOOTLOADER_ADDR;
  __set_MSP(boot_stack);
  __enable_irq();
  boot_jump();

  while (1)
  {
  }
}

void DFU_CheckCommand(void)
{
  uint8_t rx[32];
  uint16_t len = UART_Read(rx, sizeof(rx));

  for (uint16_t i = 0U; i < len; i++)
  {
    char c = DFU_ToLower((char)rx[i]);

    if (DFU_IsSeparator(c) != 0U)
    {
      DFU_HandleCommand();
      continue;
    }

    if (s_dfu_cmd_len >= (DFU_CMD_BUF_LEN - 1U))
    {
      DFU_ResetCommand();
    }

    s_dfu_cmd[s_dfu_cmd_len] = c;
    s_dfu_cmd_len++;

    if (DFU_CommandIsDfu() != 0U)
    {
      DFU_HandleCommand();
    }
  }
}

void DFU_CheckBootRequest(void)
{
  if (DFU_BootMagicIsSet() != 0U)
  {
    DFU_ClearBootMagic();
    DFU_JumpNow();
  }
}

void DFU_EnterBootloader(void)
{
  (void)HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
  (void)USBD_Stop(&hUsbDeviceFS);
  (void)USBD_DeInit(&hUsbDeviceFS);
  DFU_DelayMs(300U);

  DFU_SetBootMagic();
  NVIC_SystemReset();
}
