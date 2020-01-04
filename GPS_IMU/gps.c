/**
  ******************************************************************************
  * File Name          : gps.c
  * Description        : Get raw data from GPS Satellite, then process and get coordinate
  ******************************************************************************
  */

/**
 * author Thanh Ta Minh, Ho Chi Minh University of Technology, Vietnam
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "gps.h"
#include "math.h"
#include "define.h"
#include "stanley.h"
#include <stdbool.h>

uint8_t xbuffer[RXBUFFER];
uint8_t IMUbuffer[RXBUFFER];
uint8_t orginal_string[RXBUFFER];
uint8_t long_data [11];
uint8_t lat_data[12];
uint8_t angle[7]="000.00";
uint8_t imuuu[7];
uint8_t GSA[100];
uint8_t sai_so[4];
uint8_t temp[14]={0},temp1[16]={0},temp2[20]={0},temp3[27]={0};
uint8_t IMU_send[7];
uint8_t steering_send[7];
uint8_t turn_send[2];
uint8_t index_send[4];
double kinh_do, vi_do ;
double Imu_gt;
double Imu_real;
double Imu_real_calib;
double angle_heading;
double true_Lat =0 , true_Long=0 , true_angle = 0,mean = 0,true_Lat1=0,true_Long1=0;
uint8_t tempp,tempLat,tempLong,tempangle;
uint8_t count;
uint8_t check_GGA,check_VTG,check_GSA;
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart1_rx;
double test_test=0;



void GPS_Reading_Data(uint16_t ui16_start_index, uint16_t ui16_end_index)
{
	uint16_t index ;
	double a,b,c,d,e;
	for (index = ui16_start_index; index <= ui16_end_index; index++)
		{
				if (xbuffer[index] == '$' && xbuffer[index+1] == 'G'&& xbuffer[index+2] == 'N'&& xbuffer[index+3] == 'G'&& xbuffer[index+4] == 'G'&& xbuffer[index+5] == 'A')
				{
				 	
						for(int k =0 ; k<11;k++)
						{
							lat_data[k]= xbuffer[index+30+k] ;
						}
						kinh_do = (lat_data[0]-0x30)*100 + (lat_data[1]-0x30)*10 + (lat_data[2]-0x30)*1+ (lat_data[3]-0x30)*0.1+ (lat_data[4]-0x30)*0.01+ (lat_data[6]-0x30)*0.001+ (lat_data[7]-0x30)*0.0001+ (lat_data[8]-0x30)*0.00001+ (lat_data[9]-0x30)*0.000001+ (lat_data[10]-0x30)*0.0000001;
						a = (kinh_do -106.0)*100 ;
						true_Lat1 = a/60.0 + 106.0 ;
						c = true_Lat1;
						if(102.0<true_Lat1 && true_Lat1<110.0)
						{
							if(c==true_Lat1)
							{
							true_Lat = true_Lat1 ;
							}
						}
						lat_data[11]='B';
						
						for (int j=0 ; j<10; j++)
						{
							long_data[j] = xbuffer[index+17+j];
						}
						
						vi_do   = (long_data[0]-0x30)*10 + (long_data[1]-0x30)*1 + (long_data[2]-0x30)*0.1+ (long_data[3]-0x30)*0.01+ (long_data[5]-0x30)*0.001+ (long_data[6]-0x30)*0.0001+ (long_data[7]-0x30)*0.00001+ (long_data[8]-0x30)*0.000001+ (long_data[9]-0x30)*0.0000001;
						b = (vi_do -10)*100;
						true_Long1 = (b/60.0) +10;
						d = true_Long1;
						if(5.0<true_Long1 && true_Long1<15.0)
						{
							if(d==true_Long1)
							{
							true_Long = true_Long1 ;
							}	
						}
						long_data[10]='B';
						check_GGA =1;
				}
				if (xbuffer[index] == '$' && xbuffer[index+1] == 'G'&& xbuffer[index+2] == 'N'&& xbuffer[index+3] == 'V'&& xbuffer[index+4] == 'T'&& xbuffer[index+5] == 'G'&& xbuffer[index+7] != ','  )
				{
						for(int n=0 ; n<7 ; n++)
						{
						angle[n] = xbuffer[index+7+n];
						}
						
				angle_heading = (angle[0]-0x30)*100 +(angle[1]-0x30)*10+(angle[2]-0x30)+(angle[4]-0x30)*0.1+(angle[5]-0x30)*0.01;
				e = angle_heading ;
				if(0.0<= angle_heading && angle_heading<360.0)
					{
						if(e == angle_heading)
						{
							true_angle = angle_heading ;
						}
					}
				angle[6]='B';
				check_VTG =1;
				} 
		if (xbuffer[index] == '$' && xbuffer[index+1] == 'G' && xbuffer[index+2] == 'N' && xbuffer[index+3] == 'G' && xbuffer[index+4] == 'S' && xbuffer[index+5] == 'A')
		{
					for (int m = 0  ; m<10 ; m++)
					{
							GSA[m]=xbuffer[index+m];
						
							if(GSA[m] ==',')
								{
								count=count + 1;
								}
				
							if(count == 15)
								{
								for(int e=0; e<4; e++)
									{
									sai_so[e] = xbuffer[index+m+1+e];
									}
								count = 0;
								m= 254;
								}
							tempp =1;
					}
				mean = (sai_so[0]-0x30)+ (sai_so[2]-0x30)*0.1+(sai_so[3]-0x30)*0.01;
				check_GSA = check_GSA + 1;			
		}
//		
		}
	//	upload();
}
void IMU_Reading_Data(uint16_t ui16_start_index, uint16_t ui16_end_index)
{
	uint16_t index ;
	for (index = ui16_start_index; index <= ui16_end_index; index++)
		{
			
			if (IMUbuffer[index] == 0x0D && IMUbuffer[index+1] == 0x0A)
			{
				
				for (int j=0 ; j<7; j++)
				{			
				imuuu[j] = IMUbuffer[index+16+j];
				}
				if(imuuu[0] == 0x20)
				{
					Imu_gt = (imuuu[1]-0x30)*100+(imuuu[2]-0x30)*10+(imuuu[3]-0x30)+(imuuu[4]-0x30)*0.1+(imuuu[5]-0x30)*0.01;
				}
				if(imuuu[0] == 0x2D)
				{
					Imu_gt = (-1)*((imuuu[1]-0x30)*100+(imuuu[2]-0x30)*10+(imuuu[3]-0x30)+(imuuu[4]-0x30)*0.1+(imuuu[5]-0x30)*0.01);
				}
			}
		}
							if ( -180 <= Imu_gt && Imu_gt <= 180 )
						{
							if (Imu_gt < 0)
								Imu_real = Imu_gt+ 360.0;
							else
								Imu_real = Imu_gt;
						}
	Imu_real_calib= IMU_CALIB(Imu_real);
Double2Str(Imu_real_calib,6,IMU_send);

}
/*Devide 360 degree to 8 religion*/
double IMU_CALIB ( double IMU_REAL)
{
	if ( IMU_REAL >= IMU_calib_a1 &&  IMU_REAL <= IMU_calib_a2)
		IMU_REAL = (45.0-0.0)/(IMU_calib_a2-IMU_calib_a1)*(IMU_REAL-IMU_calib_a1) + 0;
	else if ( IMU_REAL > IMU_calib_a2 && IMU_REAL <= IMU_calib_a3)
		IMU_REAL = (90.0-45.0)/(IMU_calib_a3-IMU_calib_a2)*(IMU_REAL-IMU_calib_a2)+45.0;
	else if (IMU_REAL > IMU_calib_a3  && IMU_REAL <= IMU_calib_a4)
		IMU_REAL = (135.0-90.0)/(IMU_calib_a4-IMU_calib_a3)*(IMU_REAL-IMU_calib_a3)+90.0;
	else if (IMU_REAL > IMU_calib_a4 && IMU_REAL <= IMU_calib_a5)
		IMU_REAL = (180.0-135.0)/(IMU_calib_a5-IMU_calib_a4)*(IMU_REAL-IMU_calib_a4)+135.0;
	else if ( IMU_REAL >IMU_calib_a5 && IMU_REAL <=IMU_calib_a6)
		IMU_REAL = (225.0-180.0)/(IMU_calib_a6-IMU_calib_a5)*(IMU_REAL-IMU_calib_a5)+180.0;
	else if ( IMU_REAL >IMU_calib_a6 && IMU_REAL <=IMU_calib_a7)
		IMU_REAL = (270.0-225.0)/(IMU_calib_a7-IMU_calib_a6)*(IMU_REAL-IMU_calib_a6)+225.0;
	else if ( IMU_REAL >IMU_calib_a7 && IMU_REAL <=IMU_calib_a8)
		IMU_REAL = (315.0-270.0)/(IMU_calib_a8-IMU_calib_a7)*(IMU_REAL-IMU_calib_a7)+270.0;
	else if ( IMU_REAL >IMU_calib_a8 && IMU_REAL <360.0)
		IMU_REAL = (360.0-315.0)/(IMU_calib_a1+360.0-IMU_calib_a8)*(IMU_REAL-IMU_calib_a8)+315.0;
	else if ( IMU_REAL >= 0 && IMU_REAL < IMU_calib_a1)
	{
		IMU_REAL = IMU_REAL + 360.0;
		IMU_REAL = (360.0-315.0)/(IMU_calib_a1+360.0-IMU_calib_a8)*(IMU_REAL-IMU_calib_a8)+315.0;
	}
	else
		IMU_REAL = 1000.0;

//	if ( IMU_REAL >= 10 &&  IMU_REAL <= 85)
//		IMU_REAL = (90.0-0.0)/(85.0-10.0)*(IMU_REAL-10.0) + 0;
//	else if ( IMU_REAL > 85 && IMU_REAL <= 171)
//		IMU_REAL = (180.0-90.0)/(171.0-85.0)*(IMU_REAL-85.0)+90.0;
//	else if (IMU_REAL > 171 && IMU_REAL <= 269)
//		IMU_REAL = (270.0-180.0)/(269.0-171.0)*(IMU_REAL-171.0)+180.0;
//	else if (IMU_REAL > 269 && IMU_REAL < 360)
//		IMU_REAL = (360.0-270.0)/(370.0-269.0)*(IMU_REAL-269.0)+270.0;
//	else if ( IMU_REAL >=0 && IMU_REAL <10)
//	{
//		IMU_REAL = IMU_REAL + 360.0;
//		IMU_REAL = (360.0-270.0)/(370.0-269.0)*(IMU_REAL-269.0)+270.0;
//	}
	return IMU_REAL;

}
double Get_IMUHeading()
{
	return Imu_real_calib;
}
double Get_LongGPS(void)
{
	//return 106.659266800;
	return true_Lat;
}
double Get_LatGPS(void)
{
	//return 10.772842050;
	return true_Long;
}
//void GPS_Reading_Data(uint16_t ui16_start_index, uint16_t ui16_end_index)
//{
//	uint16_t index ;
//	for (index = ui16_start_index; index <= ui16_end_index; index++)
//		{
//			if (xbuffer[index] == 0x0D)
//			{
//				 	{
//			for (int j=0 ; j<7; j++)
//			{
//							long_data[j] = xbuffer[index+16+j];
//			}
//			for(int k =0 ; k<6;k++)
//			{
//							lat_data[k]= xbuffer[index+15+k];
//			}
//		kinh_do = lat_data[0] + lat_data[1]*0.1+ lat_data[2]*0.01+ lat_data[3]*0.001+ lat_data[4]*0.0001+ lat_data[5]*0.00001;
//		vi_do   = long_data[0]*1000 + long_data[1]*100 + long_data[2]*10+ long_data[3]*1+ long_data[5]*0.1+ long_data[6]*0.01+ long_data[7]*0.001+ long_data[8]*0.0001+ long_data[9]*0.00001;
//		}
//			}
//			
//				
//			
//		}
//}
void  GPS_Pre_Process(void)
{
	static uint16_t ui16_end_index, ui16_start_index = 0;
	if (DMA1_Stream2->NDTR == 900)
		ui16_end_index = 900 - 1;
	else
		ui16_end_index = 900 - DMA1_Stream2->NDTR - 1; 	
	if(ui16_end_index < ui16_start_index)
	{
		GPS_Reading_Data(ui16_start_index, (900 - 1));
		GPS_Reading_Data(0, ui16_end_index);
		if (ui16_end_index == (900 - 1))
			ui16_start_index = 0;
		else
			ui16_start_index = ui16_end_index + 1;
	}
	else if ((ui16_end_index > ui16_start_index) && !((ui16_start_index == 0) && (ui16_end_index == (900 - 1))))
	{
		GPS_Reading_Data(ui16_start_index, ui16_end_index);
		if (ui16_end_index == (900 - 1))
			ui16_start_index = 0;
		else
			ui16_start_index = ui16_end_index + 1;
	}
}
void  IMU_Pre_Process(void)
{
	static uint16_t ui16_end_index, ui16_start_index = 0;
	if (DMA2_Stream2->NDTR == 950)
		ui16_end_index = 950 - 1;
	else
		ui16_end_index = 950 - DMA2_Stream2->NDTR - 1; 	
	if(ui16_end_index < ui16_start_index)
	{
		IMU_Reading_Data(ui16_start_index, (950 - 1));
		IMU_Reading_Data(0, ui16_end_index);
		if (ui16_end_index == (950 - 1))
			ui16_start_index = 0;
		else
			ui16_start_index = ui16_end_index + 1;
	}
	else if ((ui16_end_index > ui16_start_index) && !((ui16_start_index == 0) && (ui16_end_index == (950 - 1))))
	{
		IMU_Reading_Data(ui16_start_index, ui16_end_index);
		if (ui16_end_index == (950 - 1))
			ui16_start_index = 0;
		else
			ui16_start_index = ui16_end_index + 1;
	}
}
void upload(void)
{				
		//if(check_GSA ==1 && check_VTG == 1 )
		//{
		IndexTOstr(index_send,3,Parameter.Index);
		if(i8_TURN_RIGHT == 0x01)
		{
			turn_send[0]='1';
		}
		else 
		turn_send[0]='0';
		Double2Str(Parameter.Steering_Angle,6,steering_send);
		
		
		
		String_Merger(steering_send,IMU_send,',',temp);
		String_Merger(temp,turn_send,',',temp1);
		String_Merger(temp1,index_send,',',temp2);
		String_Merger(temp2,angle,',',temp3);



		//String_Merger(temp,angle,',',temp1);
		HAL_UART_Transmit(&huart2,temp3,27,50);
		test_test=test_test+0.1;
		//}
//	check_GSA = 0;
//	check_VTG = 0;
//	check_GGA = 0;

}
void String_Merger(uint8_t* strfirst,uint8_t* strsecond,uint8_t character,uint8_t* stringout)
{
	uint8_t i = 0, index = 0;
	//clear old data
	while(stringout[i] != '\0')
	{
		stringout[i] = 0;
		i++;
	}		
	//merger
	i = 0;
	//long_data[10]='B';
	//	lat_data[11]='B';
		turn_send[1]='B';
		IMU_send[6]='B';
		index_send[3]='B';
		steering_send[6] = 'B';
		angle[6]='B';
		//temp[13]='B';
		//temp1[15]='B';
		//temp2[19]='B';
	while(strfirst[i] != 'B')
	{
		stringout[i] = strfirst[i];
		i++;
	}
	index = i;
	i = 0;
		stringout[index] = character;
		index++;
	
	while(strsecond[i] != '\0')
	{
		stringout[index+i] = strsecond[i];
		i++;
	}
	i = 0;
	index = 0;
}
void Int2Str(int32_t inum, uint8_t* string)
{
	bool sign = false; //possitive
	uint8_t numcount = 0, i = 0, j = 0;
	uint32_t temp = 0, posnum = 0, pownum = 0;
	while( string[i] != '\0')
	{
	  string[i] = 0;  //clear string data
		i++;
	}
	i = 0;
	if(inum>=0)//positive
	{
		posnum = inum;
		sign = false;
	}
	else     //negative
	{
		posnum = -inum;
		sign = true;
		*(string++) = '-';
	}
	temp = posnum;
	while(temp > 0)
	{
		numcount++;
		temp = temp/10; // divide and take decimal
	}
	temp = posnum;
	if(numcount == 0) // ~ inum = 0
	{
		*string = '0';
	}
	else
	{
		for(i=0;i<numcount;i++)
		{
			pownum = 1;
			for(j=i+1;j<numcount;j++)
			{
				pownum = pownum*10;
			}
			*(string++) = temp/pownum + 48;
			temp = temp%pownum;
		}
	}
}
void Double2Str(double dbnum,uint8_t num_of_frac,uint8_t* string)
{
	int32_t decnum = 0, powfracnum = 0;
	double fracnum = 0;
	uint8_t decstr[20] = {0}, fracstr[20] = {0};
	uint8_t i = 0, id = 0, ifr = 0;
	while(string[i] != '\0')
	{
		string[i] = 0;  //clear string data
		i++;
	}
	i = 0;
	decnum = (int)dbnum;// get decimal
	if(dbnum>=0)
	{
		fracnum = dbnum - decnum;//get fraction
	}
	else
	{
		fracnum = -(dbnum - decnum);//get fraction
	}
	if(dbnum < 0 && dbnum > -1) //ex: -0.xx;
	{
		string[id] = '-';
		id++;
		string[id] = '0';
		id++;
	}
	Int2Str(decnum,decstr);
	powfracnum = (uint32_t)(fracnum*pow(10.0,num_of_frac));//convert fraction to decimal with num of fraction
	Int2Str(powfracnum,fracstr);//convert fraction to string
	while(decstr[id] != '\0')// add decimal string to dest string
	{
		string[id] = decstr[id];
		id++;
	}
	string[id] = '.';//add '.'
	while(fracstr[ifr] != '\0')//add fraction string to dest string
	{
		string[id+1+ifr] = fracstr[ifr];
		ifr++;
	}	
}
void IndexTOstr(uint8_t* buffer,uint8_t size,uint8_t x )
{
	buffer[0] = (x/100.0) +0x30;
	buffer[1] = x/10 - (buffer[0]-0x30)*10+0x30;
	buffer[2] = x - (buffer[0]-0x30)*100 - (buffer[1]-0x30)*10+0x30;
}

/** System Clock Configuration
*/
//void SystemClock_Config(void)
//{

//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;

//    /**Configure the main internal regulator output voltage 
//    */
//  __HAL_RCC_PWR_CLK_ENABLE();

//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 8;
//  RCC_OscInitStruct.PLL.PLLN = 336;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 4;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Configure the Systick interrupt time 
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

//    /**Configure the Systick 
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
//}

///* UART4 init function */
 void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
//	huart4.pRxBuffPtr = (uint8_t*)xbuffer;
//	huart4.RxXferSize =RXBUFFER;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
void MX_USART1_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
void MX_USART2_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

///** 
//  * Enable DMA controller clock
//  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	//HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  //HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

///** Configure pins as 
//        * Analog 
//        * Input 
//        * Output
//        * EVENT_OUT
//        * EXTI
//*/


///* USER CODE BEGIN 4 */

///* USER CODE END 4 */

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @param  None
//  * @retval None
//  */
//void _Error_Handler(char * file, int line)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  while(1) 
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */ 
//}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 


