/***********************************
-  PPP_init.c
-		外设初始化
************************************/
#include "PPP_init.h"



sensor_state_TypeDef sensor_state;
sensor_data_TypeDef sensor_data;	
time_TypeDef time;
UsartData_TypeDef UsartData;


/********************
- RCC初始化
*********************/
void RCC_Configuration(void)
{
	ErrorStatus HSEStartUpStatus;
	
	// RCC system reset(for debug purpose)
	RCC_DeInit();
	
	// Enable HSE 
	RCC_HSEConfig(RCC_HSE_ON);
	
	// Wait till HSE is ready 
	HSEStartUpStatus = RCC_WaitForHSEStartUp();
	
	if(HSEStartUpStatus == SUCCESS)
		{
		// Enable Prefetch Buffer 
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
		
		// Flash 2 wait state 
		FLASH_SetLatency(FLASH_Latency_2);
		
		// HCLK = SYSCLK 
		RCC_HCLKConfig(RCC_SYSCLK_Div1); 
		
		// PCLK2 = HCLK/8 
		RCC_PCLK2Config(RCC_HCLK_Div8); 
		
		// PCLK1 = HCLK/2 
		RCC_PCLK1Config(RCC_HCLK_Div2);
		
		// ADCCLK = PCLK2/8 
		RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
  
		// PLLCLK = 8MHz * 9 = 72 MHz 
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9);
		
		// Enable PLL  
		RCC_PLLCmd(ENABLE);
		
		// Wait till PLL is ready 
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
			{}
		
		// Select PLL as system clock source 
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
		
		// Wait till PLL is used as system clock source 
		while(RCC_GetSYSCLKSource() != 0x08)
			{}
		}
/**/	
	// Enable peripheral clocks --------------------------------------------------
	// Enable DMA1 clock 
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	// Enable ADC1 and GPIOC clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE );
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

}

/********************
- USART1初始化
*********************/
void USART1_Configuration(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
		
	// Enable GPIO clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

	// Configure USART Tx as alternate function push-pull 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    
	// Configure USART Rx as input floating 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	// USART configuration
	USART_Init(USART1,&USART_InitStructure);

	// enable NVIC	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// USART interrupt enable
 	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);	//
 	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	// Enable USART
	USART_Cmd(USART1, ENABLE);
}

/********************
- I2C2初始化
*********************/
void SENSOR_I2C_BUS_Configuration(void)
{
	I2C_InitTypeDef I2C_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	// I2C Periph clock enable 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);   
 
	// GPIO Periph clock enable 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 

	// I2C configuration
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0xa0;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C2_Speed;
	
	// Configure I2C2 SCL as alternate function Open-drain output 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	// Configure I2C2 SDA as alternate function Open-drain output  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// Apply I2C configuration after enabling it
	//I2C_AcknowledgeConfig(SENSOR_I2C_BUS, ENABLE);
	I2C_Init(SENSOR_I2C_BUS, &I2C_InitStructure);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// I2C Peripheral Enable
	I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_EVT|I2C_IT_ERR, ENABLE);
	I2C_DMACmd(SENSOR_I2C_BUS, DISABLE);
	I2C_Cmd(SENSOR_I2C_BUS, ENABLE);
}

/********************
- DMA_I2C2初始化
*********************/
void SENSOR_I2C_BUS_DMA_Configuration(void)
{
    DMA_InitTypeDef DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
/*
	// I2C2_TX DMA1通道4
	DMA_DeInit(DMA1_Channel4);											
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;			//USART2_DR 地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&I2C2_TX_DATA[0];			        //存储器地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;								//数据方向 0 从外设读
	DMA_InitStructure.DMA_BufferSize = 100;											//
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设指针   不增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							//存储器指针 增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    	//数据大小 8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;				//
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;									//不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;									//M2M
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);	
	
	DMA_Cmd(DMA1_Channel4, ENABLE);*/

	// I2C2_RX DMA1通道5
	DMA_DeInit(DMA1_Channel5);											
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)I2C2_DR_Address;			//USART2_DR 地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&sensor_data.GYRO_OUT_X_L;    			//存储器地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;								//数据方向 0 从外设读
	DMA_InitStructure.DMA_BufferSize = 6;											//
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设指针   不增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							//存储器指针 增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	    	//数据大小 8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;				//
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;									//不循环
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;									//M2M
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel5, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
}

/********************
- LCD初始化
*********************/
void LED_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	// 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// close LED
	LED_OFF;
}

/********************
- 传感器IO中断设置
*********************/
void SENSOR_EXTI_Configuration(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, DISABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);//PB1 ACCER_INT2
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//PA6 GYRO_DR

	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); 	
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB1 ACCER_INT2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PA6 GYRO_DR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	EXTI_DeInit();
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
//	EXTI_GenerateSWInterrupt(EXTI_Line6);

	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//参考手册9.2.5节
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;	
	EXTI_Init(&EXTI_InitStructure);
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line6;//参考手册9.2.5节
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt; 
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	EXTI_GenerateSWInterrupt(EXTI_Line1|EXTI_Line6);

}

/********************
- systick设置
*********************/
void SYSTICK_Configuration(void)
{
	//SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
	if(SysTick_Config(SYSTICK_TICK))
		{
//		printf("systick error.tick is too big!\n");
		while(1);
		}
}

/********************
- 给sensor_state结构体赋值
	成功返回SUCCERS 失败返回error
*********************/
ErrorStatus sensor_state_assign(uint8_t sensor_address, uint8_t SUB)
{
	if(sensor_state.I2C_bus_busy)
		return ERROR;

	sensor_state.sensor_address = sensor_address;
	sensor_state.SUB = SUB;
	sensor_state.data_direction = I2C_Direction_Receiver;
	sensor_state.SUB_Transmitted = RESET;
	sensor_state.receive_data = 0;

	return SUCCESS;
}

/*-------------------
- USART中断服务程序
-------------------*/
void USART1_ISR(void)
{
//	uint8_t count;
	if(USART_GetITStatus(USART1, USART_IT_RXNE))//接收寄存器非空
		{
		UsartData.RxBuffer[UsartData.RxCount]=(USART_ReceiveData(USART1));
		if((UsartData.RxCount++) == USART_RXBUFFER_SIZE-1)	{UsartData.RxCount=0;}
		}
	if(USART_GetITStatus(USART1, USART_IT_TXE))//发送寄存器空
		{
		USART_SendData(USART1, UsartData.TxBuffer[UsartData.TxCount]);
		if((UsartData.TxCount++) == USART_TXBUFFER_SIZE-1)	
			{
			UsartData.TxCount=0;
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);	//关掉串口发送寄存器空 中断
			}
		}
//	for(count=0;count<USART_RXBUFFER_SIZE;count++)
//		{
//		UsartData.TxBuffer[count]=UsartData.RxBuffer[count];
//		}
}

/*-------------------
- 传感器I2C总线的事件中断服务程序
-------------------*/
void SENSOR_I2C_BUS_EV_ISR(void)
{
	//-SB
	if(I2C_GetITStatus(SENSOR_I2C_BUS, I2C_IT_SB))
		{
		I2C_AcknowledgeConfig(SENSOR_I2C_BUS, DISABLE);
		if((sensor_state.data_direction==I2C_Direction_Receiver) && sensor_state.SUB_Transmitted)
			{
			sensor_state.SUB_Transmitted = SET;
			I2C_Send7bitAddress(SENSOR_I2C_BUS, sensor_state.sensor_address, I2C_Direction_Receiver);
			}
		else
			{
			I2C_Send7bitAddress(SENSOR_I2C_BUS, sensor_state.sensor_address, I2C_Direction_Transmitter);
			}
		}
	
	//-ADDR
	else if(I2C_GetITStatus(SENSOR_I2C_BUS, I2C_IT_ADDR))
		{
		volatile uint8_t a=SENSOR_I2C_BUS->SR1;//Read SR1,2 to clear ADDR
		a=SENSOR_I2C_BUS->SR2;
		if(sensor_state.data_direction==I2C_Direction_Receiver && sensor_state.SUB_Transmitted) //已发送SUB,开始接收1字节[ref manual P735]
			{
			I2C_AcknowledgeConfig(SENSOR_I2C_BUS, DISABLE);//turn off ACK
			I2C_GenerateSTOP(SENSOR_I2C_BUS,ENABLE);//program the stop
			I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF, ENABLE);
			}
		else //接收多于3字节;发送SUB或正在发送
			{
			I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF, ENABLE);
			}
		}/*
	//-BTF										
	else if(I2C_GetITStatus(SENSOR_I2C_BUS, I2C_IT_BTF))
		{
		volatile uint8_t b=SENSOR_I2C_BUS->SR1;
		if(sensor_state.data_direction==I2C_Direction_Receiver && sensor_state.SUB_Transmitted)
			{}
		else
			{
			I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);
			}
		}*/
	
	//-RXNE
	else if(I2C_GetITStatus(SENSOR_I2C_BUS, I2C_IT_RXNE))
		{
		sensor_state.receive_data=I2C_ReceiveData(SENSOR_I2C_BUS);
		I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF, DISABLE);
		sensor_state.data_receive_finish=SET;
		}
	
	//-TXE
	else if(I2C_GetITStatus(SENSOR_I2C_BUS, I2C_IT_TXE))
		{
		I2C_SendData(SENSOR_I2C_BUS, sensor_state.SUB);
		sensor_state.SUB_Transmitted=SET;
		if(sensor_state.data_direction==I2C_Direction_Receiver)
			I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF, DISABLE);
		I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);
		}
/**/
	if(sensor_state.data_receive_finish)
		{
		uint8_t temp=(sensor_state.I2C_run_flag&0x07);//读出偏移量(低3位)
		
		sensor_state.data_receive_finish=RESET;
		sensor_state.SUB_Transmitted=RESET;

		switch(sensor_state.I2C_run_flag&0xF0)		//改成if模式的 用与判断?????????????????
			{
			case ACCER_READING:
				{
				(*(ACCER_DATA_BASE+temp))=sensor_state.receive_data;
				}
				break;
			case GYRO_READING:
				{
				(*(GYRO_DATA_BASE+temp))=sensor_state.receive_data;
				}
				break;
			case MEGN_READING://reserve!
				{
				}
				break;
			default:
				break;
			}
/*		if(sensor_state.I2C_run_flag&ACCER_READING)
			{
			(*(ACCER_DATA_BASE+temp))=sensor_state.receive_data;
			}
		else if(sensor_state.I2C_run_flag&GYRO_READING)
			{
			(*(GYRO_DATA_BASE+temp))=sensor_state.receive_data;
			}
		else if(sensor_state.I2C_run_flag&MEGN_READING)
			{}*/
		
		
		if(temp<5)//未读够6个字节数据(0:5)
			{
			sensor_state.I2C_run_flag++;//接收完一个字节,标志加1
			sensor_state.SUB++;
			sensor_state.SUB_Transmitted=RESET;
			I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);
			}
		else//接收到第6个字节
			{
//			sensor_state.I2C_run_flag=0;//复位所有运行标志
			sensor_state.I2C_run_flag-=0x10;
			if((sensor_state.I2C_run_flag&0xF0)==ACCER_READING)
				{
				}
			else if((sensor_state.I2C_run_flag&0xF0)==GYRO_READING)
				{						
				sensor_state_assign(GYRO_address, OUT_X_L_G);
				sensor_state.I2C_run_flag=(GYRO_READING);
				I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);	
				}
//			else if((sensor_state.I2C_run_flag&0xF0)==MEGN_READING)
//				{
//				}
			else
				{
				sensor_state.I2C_run_flag=0;
				I2C_GenerateSTOP(SENSOR_I2C_BUS, ENABLE);
				}
			}		
		}
}

/********************
- 传感器I2C总线的错误中断服务程序
*********************/
void SENSOR_I2C_BUS_ERR_ISR(void)
{
	__IO uint32_t SR1Register, SR2Register;
	/* Read the I2C1 status register */
	SR1Register = SENSOR_I2C_BUS->SR1;
	if(SR1Register & 0x0F00) 
		{
		}
	/* If AF or BERR, send STOP*/
	if(SR1Register & 0x0500)
		I2C_GenerateSTOP(SENSOR_I2C_BUS, ENABLE);//program the Stop
	/* If AF, BERR or ARLO, abandon the current job and send a start to commence new */
	if(SR1Register & 0x0700) 
		{
		SR2Register = I2C1->SR2;//read second status register to clear ADDR if it is set (note that BTF will not be set after a NACK)
		I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF, DISABLE);
		I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);//sets a start (ref manual p743 - appears ok as long as stop and start are seperate writes) 
		}
	SENSOR_I2C_BUS->SR1 &=~0x0F00;		//reset all the error bits to clear the interrupt

	sensor_state.data_receive_finish=SET;
}

/********************
- systick中断服务程序
*********************/
void SYSTICK_ISR(void)
{
	time.time_tamp++;
	if((time.time_tamp%400)==0)	{time.led=SET;}
	if((time.time_tamp%5)==0)	{time.sensor_read=SET;}
}

