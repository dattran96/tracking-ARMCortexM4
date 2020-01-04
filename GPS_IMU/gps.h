
#ifndef __GPS_H
#include "stm32f4xx_hal.h"
#define __GPS_H
#define RXBUFFER 1024
extern	UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_uart1_rx;
extern uint8_t xbuffer[RXBUFFER];
extern uint8_t IMUbuffer[RXBUFFER];
extern UART_HandleTypeDef huart2;
extern  uint8_t long_data[11],lat_data[12],angle[7];
extern uint8_t temp[14],temp1[16],temp2[20],temp3[27];
extern uint8_t check_GGA,check_VTG,check_GSA;
 void GPS_Pre_Process(void);
 void IMU_Pre_Process(void);
 void MX_USART1_Init(void);
 void MX_USART2_Init(void);
 void MX_UART4_Init(void);
 void MX_DMA_Init(void) ;
 double Get_IMUHeading(void);
 double IMU_CALIB ( double IMU_REAL);
 double Get_LatGPS(void);
double Get_LongGPS(void);
void String_Merger(uint8_t* strfirst,uint8_t* strsecond,uint8_t character,uint8_t* stringout);
void Int2Str(int32_t inum, uint8_t* string);
void Double2Str(double dbnum,uint8_t num_of_frac,uint8_t* string);
void IndexTOstr(uint8_t* buffer,uint8_t size,uint8_t x );

void upload(void);

#endif  /*__GPS_H*/ 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
