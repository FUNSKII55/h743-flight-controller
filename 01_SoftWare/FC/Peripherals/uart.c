#include "uart.h"

#include <stdio.h>
#include <string.h>
#include "../Inc/usbd_cdc_if.h"

void UART_Init(void)
{
  /* USB CDC is already started during system init. */
}

uint8_t UART_IsConnected(void)
{
  return CDC_IsConnected();
}

uint8_t UART_SendBuffer(const uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0U))
  {
    return 0U;
  }

  if (!UART_IsConnected())
  {
    return 0U;
  }

  return (CDC_Transmit_FS((uint8_t *)buf, len) == USBD_OK) ? 1U : 0U;
}

uint8_t UART_SendText(const char *text)
{
  if (text == NULL)
  {
    return 0U;
  }

  return UART_SendBuffer((const uint8_t *)text, (uint16_t)strlen(text));
}

uint8_t UART_SendAttitude(int16_t pitch_cd, int16_t roll_cd, int16_t yaw_cd, uint8_t who_am_i)
{
  char tx[64];
  int len = 0;

  if (who_am_i == 0x47U)
  {
    len = snprintf(tx, sizeof(tx), "%d,%d,%d\r\n", pitch_cd, roll_cd, yaw_cd);
  }
  else
  {
    len = snprintf(tx, sizeof(tx), "ERR,%02X\r\n", who_am_i);
  }

  if (len <= 0)
  {
    return 0U;
  }

  return UART_SendBuffer((const uint8_t *)tx, (uint16_t)len);
}

uint16_t UART_Read(uint8_t *buf, uint16_t max_len)
{
  if ((buf == NULL) || (max_len == 0U))
  {
    return 0U;
  }

  return CDC_ReadRxData(buf, max_len);
}
