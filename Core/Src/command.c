#include "command.h"
#include "usart.h"     
#include <string.h>

static uint8_t  rx_dma_buf[COMMAND_DMA_BUF_SIZE];    // DMA 硬件直接写入的缓冲区
static uint16_t rx_old_pos = 0;                      // 上一次处理到的位置，用来算"这次新收了多少字节" 

static uint8_t  ring_buf[COMMAND_RING_SIZE];         // 环形缓冲区
static volatile uint16_t read_index = 0;             // 读索引
static volatile uint16_t write_index = 0;            // 写索引

void Command_Init(void)
{
    rx_old_pos  = 0;
    read_index  = 0;
    write_index = 0;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_dma_buf, COMMAND_DMA_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
}

void Command_Send(const uint8_t *data, uint16_t len)
{
    if (len == 0 || len > COMMAND_TX_BUF_SIZE) return;

    uint32_t start = HAL_GetTick();
    while (huart4.gState != HAL_UART_STATE_READY)
    {
        if (HAL_GetTick() - start > 100) return;
    }

    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)data, len);
}


static uint16_t Command_GetLength(void)
{
    return (uint16_t)((write_index + COMMAND_RING_SIZE - read_index) % COMMAND_RING_SIZE);
}

static uint16_t Command_GetRemain(void)
{
    // 留 1 字节作满/空区分
    return (uint16_t)(COMMAND_RING_SIZE - 1 - Command_GetLength());
}

static void Command_AddReadIndex(uint16_t length)
{
    read_index += length;
    read_index %= COMMAND_RING_SIZE;
}

static uint8_t Command_Read(uint16_t i)
{
    return ring_buf[(read_index + i) % COMMAND_RING_SIZE];
}

/**
  * @brief  把一批字节写进环形缓冲区
  * @param  data   数据指针
  * @param  length 字节数
  * @return 实际写入的字节数，0 表示空间不足
  * @note   在中断上下文里被调用（由接收回调转发）
  */
uint16_t Command_Write(const uint8_t *data, uint16_t length)
{
    if (length == 0) return 0;

    // 如果缓冲区不足，则不写入数据，返回0 
    if (Command_GetRemain() < length) return 0;

    // 使用memcpy函数将数据写入缓冲区
    if (write_index + length <= COMMAND_RING_SIZE)
    {
        // 情况一：写到缓冲区末尾之前，一次拷完
        memcpy(&ring_buf[write_index], data, length);
        write_index = (uint16_t)((write_index + length) % COMMAND_RING_SIZE);
    }
    else
    {
        // 情况二：跨过末尾，分两段写（尾部 + 头部）
        uint16_t first = (uint16_t)(COMMAND_RING_SIZE - write_index);
        memcpy(&ring_buf[write_index], data, first);
        memcpy(&ring_buf[0], data + first, (uint16_t)(length - first));
        write_index = (uint16_t)(length - first);
    }

    return length;
}

/**
  * @brief  从环形缓冲区里取出一条完整、合法的指令
  * @param  command 输出缓冲区，长度至少 COMMAND_MAX_LENGTH
  * @return 指令长度（含包头和校验和），0 表示暂时没有完整指令
  *
  * 帧格式：0xAA | 长度 | 数据... | 校验和
  *   长度   = 整帧字节数，范围 3~255
  *   校验和 = 包头到数据段末尾所有字节之和的低 8 位
  */
uint8_t Command_GetCommand(uint8_t *command)
{
    // 寻找完整指令
    while (1)
    {
        uint16_t avail = Command_GetLength();

        // 如果缓冲区长度小于COMMAND_MIN_LENGTH 则不可能有完整的指令
        if (avail < COMMAND_MIN_LENGTH) return 0;

        // 如果不是包头 则跳过 重新开始寻找
        if (Command_Read(0) != COMMAND_HEAD)
        {
            Command_AddReadIndex(1);
            continue;
        }

        uint8_t length = Command_Read(1);

        // 长度非法 当前这个 0xAA 是假的，跳过重找
        if (length < COMMAND_MIN_LENGTH || length > COMMAND_MAX_LENGTH)
        {
            Command_AddReadIndex(1);
            continue;
        }

        // 如果缓冲区长度小于指令长度 则不可能有完整的指令
        if (avail < length) return 0;

        // 如果校验和不正确 则跳过 重新开始寻找
        uint8_t sum = 0;
        for (uint16_t i = 0; i < (uint16_t)(length - 1); i++)
        {
            sum = (uint8_t)(sum + Command_Read(i));
        }

        // 校验和不符 跳过 1 字节重找
        if (sum != Command_Read((uint16_t)(length - 1)))
        {
            Command_AddReadIndex(1);
            continue;
        }

        // 如果找到完整指令 则将指令写入command 返回指令长度
        for (uint16_t i = 0; i < length; i++)
        {
            command[i] = Command_Read(i);
        }
        Command_AddReadIndex(length);

        return length;
    }
}

/**
  * @brief  接收事件处理：把 DMA 新收的字节搬进环形缓冲区
  * @param  Pos DMA 当前的写入偏移
  *
  * 用「当前偏移 - 上次偏移」算出这批新增了多少字节，
  * 若绕过了缓冲区末尾则分两段拼接。
  */
static void Command_RxEventHandler(uint16_t Pos)
{
    uint16_t new_pos = (uint16_t)(Pos % COMMAND_DMA_BUF_SIZE);

    if (new_pos > rx_old_pos)
    {
        //正常写入
        Command_Write(&rx_dma_buf[rx_old_pos], (uint16_t)(new_pos - rx_old_pos));
    }
    else if (new_pos < rx_old_pos)
    {
        //尾部 + 头部两段分别写
        Command_Write(&rx_dma_buf[rx_old_pos], (uint16_t)(COMMAND_DMA_BUF_SIZE - rx_old_pos));
        Command_Write(&rx_dma_buf[0], new_pos);
    }
    else
    {
        // 位置没变 = 空帧（噪声误触发），丢弃
        return;
    }

    rx_old_pos = new_pos;
}

/**
  * @brief  HAL 弱函数重写：空闲/半满/全满事件回调
  *   UART4_IRQHandler → HAL_UART_IRQHandler → 这里
  *
  *   Circular 模式下 HAL 会在三种情况回调：
  *     HAL_UART_RXEVENT_HT    DMA 传了一半
  *     HAL_UART_RXEVENT_TC    DMA 传满了
  *     HAL_UART_RXEVENT_IDLE  总线空闲（真正的一批数据结束）
  *
  *   只有 IDLE 才代表"发送方说完了这一批"，其余是 DMA 的进度通知。
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance != UART4) return;

    // 只处理空闲事件，其余（半满/全满）忽略 
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE) return;

    Command_RxEventHandler(Size);
}
