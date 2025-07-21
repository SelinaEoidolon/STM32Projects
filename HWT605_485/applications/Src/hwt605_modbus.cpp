#include "hwt605_modbus.h"
#include "usart.h"
#include <cstdint>
#include <string.h>

IMU_t imu;
uint8_t hwt605buf[1];
 int a[11],b;
 uint16_t c;

extern "C"
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance==USART3)
    {
        Receive_IMU_Data(hwt605buf[0], 1);
    }
		 
     a[b]=hwt605buf[0];
  b++;
     if (b==11)
     {
         b=0;
     }
 		c=(0xd1<<8|0x41);
		  
}
extern "C"
void get_imu_date(void const * argument)
{
  /* USER CODE BEGIN get_imu_date */
  /* Infinite loop */
  for(;;)
  {
    Requst_IMU_Data(0x003D, 0X0003);
    // Receive_IMU_Data(hwt605buf[0], 1);
    osDelay(5);
  }
  /* USER CODE END get_imu_date */
}

void Requst_IMU_Data(uint16_t ADDR , uint16_t LEN)
{
    uint8_t tx_buffer[8]={0};

    uint8_t IMU_READ =0X03;
    uint8_t MODBUS_ID=0X50;

	uint8_t ADDRH, ADDRL;
	uint8_t LENH, LENL;
	uint8_t CRCH, CRCL;

    uint16_t CRC16;

    ADDRH = ADDR >> 8; ADDRL = ADDR;
    LENH  = LEN  >> 8; LENL  = LEN;

    tx_buffer[0] = MODBUS_ID;
	tx_buffer[1] = IMU_READ;
	tx_buffer[2] = ADDRH;
	tx_buffer[3] = ADDRL;
	tx_buffer[4] = LENH;
	tx_buffer[5] = LENL; 

    CRC16 = Check_CRC16(tx_buffer,6);
    CRCH  = CRC16 >> 8; CRCL = CRC16;

    tx_buffer[6] = CRCH;
    tx_buffer[7] = CRCL;

     //HAL_UART_Transmit_DMA(&huart3, tx_buffer, 8);
    HAL_UART_Transmit(&huart3, tx_buffer, 8, 100);

}

uint16_t crc16_modbus_direct(const uint8_t *data, uint16_t length) 
{
    uint16_t crc = 0xFFFF;  // 初始值

    for (uint16_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i];  // 异或当前字节

        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {  // 如果最低位是 1
                crc >>= 1;       // 右移一位
                crc ^= 0xA001;   // 异或多项式 0xA001
            } else {
                crc >>= 1;       // 右移一位
            }
        }
    }

    return crc;
}
uint16_t Check_CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
        uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}
/**
 * @brief 接收数据函数
 * 
 * @param received_byte 
 * @param Data_len 去掉ID和功能码，单纯的数据长度
 */    
void Receive_IMU_Data(uint8_t received_byte, uint8_t Data_len)
{
    static uint8_t step = 0;
    static uint8_t count = 0,Buf[300];
    static uint8_t *data_ptr;
    uint16_t CRC16,crc162;

    switch (step)
    {
        case 0:
            if (received_byte == 0x50) // 检测起始字节
            {
                step++;
                count = 0;
                Buf[count++] = received_byte;
            }
            break;

        case 1:
            if (received_byte == 0x03) // 检测第二个字节
            {
                step++;
                Buf[count++] = received_byte;
            }
            else if (received_byte == 0x50) // 重新检测起始字节
            {
                step = 1;
            }
            else // 无效字节，重置状态机
            {
                step = 0;
            }
            break;

        case 2:
            if (received_byte == 0x06) // 检测第三个字节
            {
                step++;
                Buf[count++] = received_byte;
            }
            else // 无效字节，重置状态机
            {
                step = 0;
            }
            break;

        case 3:
            data_ptr = &Buf[count]; // 记录数据部分的起始位置
            Buf[count++] = received_byte; // 存储数据字节
            if (count > 10) // 检查是否接收完所有数据
            {
                step++;
            }
            break;

        case 4:
             CRC16=(uint16_t)(Buf[9] << 8 | Buf[10]);
             crc162=Check_CRC16(Buf, 9);
            if ( CRC16 == crc162) // CRC 校验
            {
                imu.Euler.roll = ((int16_t)((int16_t)(a[3])<<8 | a[4]))/ 32768.0f * 180.0f;
                imu.Euler.pitch= ((int16_t)((int16_t)(a[5])<<8 | a[6]))/ 32768.0f * 180.0f;
                imu.Euler.yaw  = ((int16_t)((int16_t)(a[7])<<8 | a[8]))/ 32768.0f * 180.0f;
                // IMU_Data_Analysis(data_ptr); // 处理有效数据
            }
            else // CRC 校验失败，重置状态机
            {
                step = 0;
            }
            break;

        default:
            step = 0; // 未知状态，重置状态机
            break;
    }
}
void IMU_Data_Analysis(uint8_t *data)
{
    imu.Euler.roll = ((int16_t)((int16_t)(data[0])<<8 | data[1]))/ 32768.0f * 180.0f;
    imu.Euler.pitch= ((int16_t)((int16_t)(data[2])<<8 | data[3]))/ 32768.0f * 180.0f;
    imu.Euler.yaw  = ((int16_t)((int16_t)(data[4])<<8 | data[5]))/ 32768.0f * 180.0f;

	// imu.Euler.roll = imu.Euler.roll / 32768.0f * 180.0f;
	// imu.Euler.pitch = imu.Euler.pitch / 32768.0f * 180.0f;
	// imu.Euler.yaw = imu.Euler.yaw / 32768.0f * 180.0f;
}
extern "C"
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
//   Receive_IMU_Data(hwt605buf[0], 6);
  HAL_UART_Receive_IT(&huart3,hwt605buf, 1);
  /* USER CODE END USART3_IRQn 1 */
}
