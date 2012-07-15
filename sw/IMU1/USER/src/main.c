/***********************************
-  103_IMU   v0.1
-					 by:dachang
-创建:2012.1.2
************************************/

#include "conf.h"
#include "test1.c"

uint16_t test_state=0;

int main()
{
//	uint8_t a;
//	uint32_t i=0;
//	int16_t thet[3];
	
	RCC_Configuration();
	LED_Configuration();
	USART1_Configuration();
	SYSTICK_Configuration();

	SENSOR_I2C_BUS_Configuration();	
	GYRO_Init();	
	
//	kalman_init();	//给kalman的参数赋值
	kalmanFilterFloatInit();
	quaternionInit();

	UsartData.TxBuffer[0]=0x24;	//$作为串口传送的开始字符
	
	sensor_state.I2C_run_flag= 0x00;
//	sensor_state.data_receive_finish = SET;
	
	while(1)
		{
//		for(a=0;a<USART_RXBUFFER_SIZE;a++)
//			{
//			UsartData.TxBuffer[USART_CTRL_OFFSET+a] = UsartData.RxBuffer[a];
//			}
		USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		if(time.sensor_read)
			{
		//	uint8_t count=0;
			time.sensor_read=RESET;
			
			sensor_data.ACCER_X = sensor_data.ACCER_OUT_X_H*0x100 + sensor_data.ACCER_OUT_X_L; 
			sensor_data.ACCER_Y = sensor_data.ACCER_OUT_Y_H*0x100 + sensor_data.ACCER_OUT_Y_L; 
			sensor_data.ACCER_Z = sensor_data.ACCER_OUT_Z_H*0x100 + sensor_data.ACCER_OUT_Z_L; 
			sensor_data.GYRO_X = sensor_data.GYRO_OUT_X_H*0x100 + sensor_data.GYRO_OUT_X_L; 
			sensor_data.GYRO_Y = sensor_data.GYRO_OUT_Y_H*0x100 + sensor_data.GYRO_OUT_Y_L; 
			sensor_data.GYRO_Z = sensor_data.GYRO_OUT_Z_H*0x100 + sensor_data.GYRO_OUT_Z_L; 
			
			kalmanFloatTest();
			quaternionTest();

			floatToChar(&kFFloat.S[0], &UsartData.TxBuffer[0], KALMAN_P, USART_A_OFFSET);
/**/		UsartData.TxBuffer[USART_TXBUFFER_SIZE-1]=0x26; //&作为串口传送的结束字符
//			USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	//取到数据,打开串口发送中断 
			
			sensor_state_assign(ACCER_address, OUT_X_L_A);
			sensor_state.I2C_run_flag=ACCER_READING;	//从accer开始读取
			I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);	
			
			}

		if(time.led)
			{
			time.led=RESET;
			LED_TOGGLE;
			}
		
//		printf("G:%d %d %d	   ",sensor_data.GYRO_X,sensor_data.GYRO_Y,sensor_data.GYRO_Z);
//		printf("A:%d %d %d\r\n",sensor_data.ACCER_X,sensor_data.ACCER_Y,sensor_data.ACCER_Z);

		}
}

/**
  * @brief  Redefine fputc function.
  * @param  ...
  * @retval ...
  */
int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (u8) ch);

  /* Loop until the end of transmission */
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
  {
  }

  return ch;
}

