#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"

/*
 * 本模块封装的是当前工程使用的 USB CDC 虚拟串口，不是普通 UART 外设。
 *
 * 使用方法：
 * 1. 先在电脑端打开枚举出来的 COM 口。
 * 2. 如有需要，发送前先判断 UART_IsConnected()。
 * 3. 发送普通文本：
 *      UART_SendText("hello\r\n");
 * 4. 发送 FireWater 曲线数据：
 *      UART_SendAttitude(pitch_cd, roll_cd, who_am_i);
 *    其中 pitch_cd / roll_cd 为角度值乘以 100。
 * 5. 读取上位机发送的数据：
 *      len = UART_Read(buf, sizeof(buf));
 */

/* 初始化串口模块。
 * 参数：无。
 * 用法：当前工程的 USB CDC 已在系统启动时初始化，这里保留为统一入口。
 * 返回值：无。
 */
void UART_Init(void);

/* 判断电脑端是否已经打开当前虚拟串口。
 * 参数：无。
 * 返回值：
 *   1 = 已连接，可发送数据。
 *   0 = 未连接，发送通常不会成功。
 */
uint8_t UART_IsConnected(void);

/* 发送一段原始字节数据。
 * 参数：
 *   buf：待发送数据的首地址。
 *   len：待发送的字节数。
 * 用法：适合发送任意二进制数据，或已经组织好的字符串缓存。
 * 返回值：
 *   1 = 已提交发送。
 *   0 = 未发送成功。
 */
uint8_t UART_SendBuffer(const uint8_t *buf, uint16_t len);

/* 发送以 '\0' 结尾的文本字符串。
 * 参数：
 *   text：待发送的字符串指针，例如 "hello\r\n"。
 * 用法：适合发送调试文本、状态信息。
 * 返回值：
 *   1 = 已提交发送。
 *   0 = 未发送成功。
 */
uint8_t UART_SendText(const char *text);

/* 发送姿态数据，格式固定为 FireWater 文本。
 * 参数：
 *   pitch_cd：俯仰角，单位为角度 x100，例如 1234 表示 12.34 度。
 *   roll_cd：横滚角，单位为角度 x100，例如 -567 表示 -5.67 度。
 *   who_am_i：IMU 的 WHO_AM_I 读取值，用于判断传感器是否正常。
 * 用法：
 *   传感器正常时发送 "pitch,roll\r\n"。
 *   传感器异常时发送 "ERR,xx\r\n"。
 * 返回值：
 *   1 = 已提交发送。
 *   0 = 未发送成功。
 */
uint8_t UART_SendAttitude(int16_t pitch_cd, int16_t roll_cd, int16_t yaw_cd, uint8_t who_am_i);

/* 读取上位机发送过来的数据。
 * 参数：
 *   buf：接收缓存首地址。
 *   max_len：本次最多读取多少字节。
 * 用法：调用后返回实际读取字节数，数据会从内部接收缓冲区取出。
 * 返回值：
 *   实际读取到的字节数。0 表示当前没有新数据。
 */
uint16_t UART_Read(uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H__ */
