#ifndef __COMMAND_H
#define __COMMAND_H

#include "main.h"

#define COMMAND_DMA_BUF_SIZE    256   // DMA 直接写入的缓冲区大小
#define COMMAND_RING_SIZE       256   // 环形缓冲区大小
#define COMMAND_TX_BUF_SIZE     256   // 发送长度上限 

#define COMMAND_HEAD            0xAA
#define COMMAND_MAX_LENGTH      255
#define COMMAND_MIN_LENGTH      3

void Command_Init(void);
void Command_Send(const uint8_t *data, uint16_t len);
uint8_t Command_GetCommand(uint8_t *command);
uint16_t Command_Write(const uint8_t *data, uint16_t length);

#endif /* __COMMAND_H */
// Git reset demo
