/***********************************
-  103_IMU   v0.1
-					 by:dachang
-´´½¨:2012.1.2
************************************/

#include "conf.h"
#include "test1.c"

uint16_t test_state=0;

int main()
{
	uint8_t a=0;
	uint32_t i=0;
	
	RCC_Configuration();
	LED_Configuration();
	USART1_Configuration();
	SYSTICK_Configuration();
	SENSOR_I2C_BUS_Configuration();
	
//	GYRO_Init();

	SENSOR_EXTI_Configuration();


	sensor_state.I2C_run_flag= 0x00;
//	sensor_state.data_receive_finish = SET;
	
	while(1)
		{
//  		for(i=0;i<100000;i++)
//			{}		
//		LED_ON;
//	//	LED_OFF;
//		a++;
//		for(i=0;i<100000;i++)
//			{}
//		LED_OFF;
//		
//		printf("G:%d %d %d     ",sensor_data.GYRO_X,sensor_data.GYRO_Y,sensor_data.GYRO_Z);
//		printf("A:%d %d %d\n",sensor_data.ACCER_X,sensor_data.ACCER_Y,sensor_data.ACCER_Z);
//	//	printf("%x %x \n", a,b);

		a=matrix_test();
		printf("xx\n");
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

