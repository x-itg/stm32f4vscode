#include "main.h"
#include "update.h"
unsigned char RxBufU2[MAXU2MAX]; // 开辟2K字节的RAM用于缓存
unsigned char TxBufU2[MAXU2MAX];
// 纳秒延时
void delay_ns(unsigned int ndelay)
{
  unsigned int i = 0;
  for (i = 0; i < 5; i++)
  {
    ;
  }
}

// 微秒延时
void delay_us(unsigned int udelay)
{
  unsigned int i = 0;
  for (i = 0; i < udelay; i++)
  {
    delay_ns(1000);
  }
}

// 毫秒延时
void delay_ms(unsigned int nms)
{
  for (; nms > 0; nms--)
  {
    delay_us(1000);
  }
}
// 开机
void shortKeyPress(void)
{
  powerKeyRelse;
  powerKeyPress;
  delay_ms(150); //>= 60ms
  powerKeyRelse;
}
// 精确延时
void I2C_Delay_us(uint16_t t)
{
  uint16_t counter = 0;
  __HAL_TIM_SET_AUTORELOAD(&htim2, t); // 设置定时器自动加载值，到该值后重新计数
  __HAL_TIM_SET_COUNTER(&htim2, 0);    // 设置定时器初始值
  HAL_TIM_Base_Start(&htim2);          // 启动定时器
  while (counter != t)                 // 直到定时器计数从 0 计数到 us 结束循环,刚好  us
  {
    counter = __HAL_TIM_GET_COUNTER(&htim2); // 获取定时器当前计数
  }
  HAL_TIM_Base_Stop(&htim2); // 停止定时器
}
// 定义驱动函数----------------------------------------------------------------
void Bsp_WT588SDAH(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // SDA_H
}

void Bsp_WT588SDAL(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // SDA_L
}

void Bsp_WT588SCLH(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // SCL
}

void Bsp_WT588SCLL(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // SCL
}

signed char Bsp_WT588SDAValue(void)
{
  return (signed char)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5);
}
// 语音芯片的驱动 播报用到
void IIC_Send_Byte(unsigned char txd)
{
  unsigned char t;
  Bsp_WT588SCLL(); //
  delay_ms(10);

  for (t = 0; t < 8; t++)
  {
    if ((txd & 0x01) != 0x00)
    {
      Bsp_WT588SDAH();
    }
    else
    {
      Bsp_WT588SDAL();
    }
    txd >>= 1;
    I2C_Delay_us(300);
    Bsp_WT588SCLH();
    I2C_Delay_us(300);
    Bsp_WT588SCLL();
    I2C_Delay_us(2);
  }

  Bsp_WT588SDAH();
  Bsp_WT588SCLH();
}
void u2rxstrdecodeProcess(void)
{
  if ((__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) != RESET)) // 空闲中断
  {
    __HAL_UART_CLEAR_IDLEFLAG(&huart2);                 // 清除空闲中断
    HAL_UART_DMAStop(&huart2);                          // 暂停DMA
    RxLenU2 = MAXU2MAX - huart2.hdmarx->Instance->NDTR; // 接收到的字节长度
    rsPackFlag = 1;                                     // 正在接收
    rsRxTime = 0;
    unsigned short i = rsRxIndex;
    for (i = rsRxIndex; i < rsRxIndex + RxLenU2; i++)
    {
      rsRxBuf[i] = RxBufU2[i - rsRxIndex];
    }
    rsRxIndex = rsRxIndex + RxLenU2; // 长度

    if (RxBufU2[0] == 0xA0 &&
        RxBufU2[1] == 0xA1 &&
        RxBufU2[2] == 0xA2 &&
        RxBufU2[3] == 0x11 &&
        RxBufU2[4] == 0xA0 &&
        RxBufU2[5] == 0xA1 &&
        RxBufU2[6] == 0xA2 &&
        RxBufU2[7] == 0xA3)
    {
      gotoAppflag = 1;
    }
    // AT+DISC
    if (RxBufU2[0] == 'A' && RxBufU2[1] == 'T' && RxBufU2[2] == '+' && RxBufU2[3] == 'D' && RxBufU2[4] == 'I' && RxBufU2[5] == 'S' && RxBufU2[6] == 'C') // 将接收到来自主机客户端的透传指令 发给这个模块 如果不是AT指令则会 透传到上位机 如果是AT指令模块就会生效
    {
      BlePwrcH; // 下降沿唤醒
      BlePwrcL; // 下降沿唤醒
      HAL_UART_Transmit(&huart2, RxBufU2, RxLenU2, 20);
    }
    else
    {
      BlePwrcH; // 回传
    }
    // AT+BAUD9
    if (RxBufU2[0] == 'A' && RxBufU2[1] == 'T' && RxBufU2[2] == '+' && RxBufU2[3] == 'B' && RxBufU2[4] == 'A' && RxBufU2[5] == 'U' && RxBufU2[6] == 'D') // 将接收到来自主机客户端的透传指令 发给这个模块 如果不是AT指令则会 透传到上位机 如果是AT指令模块就会生效
    {
      BlePwrcH; // 下降沿唤醒
      BlePwrcL; // 下降沿唤醒
      HAL_UART_Transmit(&huart2, RxBufU2, RxLenU2, 20);
    }
    else
    {
      BlePwrcH; // 回传
    }

    HAL_UART_Receive_DMA(&huart2, RxBufU2, MAXU2MAX); // 再次开启DMA接收
  }
}
