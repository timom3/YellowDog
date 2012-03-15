/*git test*/
//更新一下包含读写 全部测试通过 

#define I2C1_CR1_PE       (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+0*4)) 
#define I2C1_CR1_START    (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+8*4)) 
#define I2C1_CR1_STOP     (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+9*4)) 
#define I2C1_CR1_ACK      (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+10*4)) 
#define I2C1_CR1_SWRST    (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+15*4)) 

#define I2C1_CR2_ITERREN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+8*4)) 
#define I2C1_CR2_ITEVTEN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+9*4)) 
#define I2C1_CR2_ITBUFEN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+10*4)) 

#define I2C2_CR1_PE       (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+0*4)) 
#define I2C2_CR1_START    (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+8*4)) 
#define I2C2_CR1_STOP     (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+9*4)) 
#define I2C2_CR1_ACK      (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+10*4)) 
#define I2C2_CR1_SWRST    (*(u32*)(0x42000000+(I2C1_BASE+0x00)*32+15*4)) 

#define I2C2_CR2_ITERREN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+8*4)) 
#define I2C2_CR2_ITEVTEN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+9*4)) 
#define I2C2_CR2_ITBUFEN  (*(u32*)(0x42000000+(I2C1_BASE+0x04)*32+10*4)) 


void I2C1Init() 
{ 
	I2C_InitTypeDef  I2C_InitStructure; 
	NVIC_InitTypeDef   NVIC_InitStructure; 
// 
//  /* I2C1 configuration ------------------------------------------------------*/ 
//  I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; 
//  I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2; 
//  I2C_InitStructure.I2C_OwnAddress1 =0; 
//  I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
//  I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
//  I2C_InitStructure.I2C_ClockSpeed =100000; 
//  I2C_Init(I2C1, &I2C_InitStructure); 
    NVIC_InitStructure.NVIC_IRQChannel =I2C2_EV_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =10; 
    NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 

    NVIC_InitStructure.NVIC_IRQChannel =I2C2_ER_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority =0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =10; 
    NVIC_InitStructure.NVIC_IRQChannelCmd= ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 

	I2C2->CR1=0; 
	I2C2->CR2=0x0024; 
    I2C2->OAR1=0x4000; 
	I2C2->OAR2=0; 
	I2C2->SR1=0; 
	I2C2->SR2=0; 
	I2C2->CCR=0x00b4; 
	I2C2->TRISE=0x0025; 
	
	I2C2_CR2_ITEVTEN=0; 
	I2C2_CR2_ITERREN=0;  
	I2C2_CR2_ITBUFEN=0; 
	I2C2_CR1_PE=1; 
} 

static u8  I2C_SLA; 
static u8  I2C_SUB; 
static u8  I2C_RunFlag; 
static u16 I2C_ByteNum; 
static u8 *I2C_DataPoint; 

void WriteDataToFM24C01(u16 sub,u8 *wrbuf, u16 len) 
{ 
	I2C_SLA=sub>255?0xA2:0xA0; 
	I2C_SUB=sub&0xff; 
	I2C_ByteNum=len; 
	I2C_DataPoint=wrbuf; 
	I2C_RunFlag=1; 
	I2C1_CR2_ITEVTEN=1; 
	I2C1_CR1_START=1; 
} 

void ReadDataFormFM24C01(u16 sub,u8 *rdbuf, u16 len) 
{ 
	//I2C_SLA=sub>255?0xA2:0xA0; 
	I2C_SLA=0xD0;
	I2C_SUB=sub&0xff; 
	I2C_ByteNum=len; 
	I2C_DataPoint=rdbuf; 
	I2C_RunFlag=3; 
	I2C1_CR2_ITEVTEN=1; 
//	I2C_GenerateSTART(SENSOR_I2C_BUS, ENABLE);
	I2C1_CR1_START=1; 
} 


void I2C1_EV_IRQHandler(void)	 //0xa2,0xa0 
{ 
	u16 SR1,SR2;	
	SR1=I2C1->SR1; 
	if(SR1&0x0001)//SB 
		{ 
		if(I2C_RunFlag&0x01) 
			{ 
			I2C1->DR=I2C_SLA; 
			I2C1_CR2_ITERREN=1;  
			} 
		else
 			{ 
			I2C1->DR=I2C_SLA|0x01;
			} 
		} 
	if(SR1&0x0002)//ADDR 
		{ 
		SR2=I2C1->SR2; 
		if(I2C_RunFlag&0x01) 
			{
			if(I2C_RunFlag&0x02)//读任务第一次写从地址 
				{ 
				I2C1->DR=I2C_SUB; 
				} 
			else//写任务的写从地址 
				{ 
				I2C1->DR=I2C_SUB; 
				I2C1_CR2_ITBUFEN=I2C_ByteNum>0?1:0;//根据要发送到数据长度决定 
 				} 
   			} 
  		else//读任务的第二次写从地址 
   			{ 
 			if(I2C_ByteNum>1)//接收多个字节 
  				{ 
				I2C1_CR1_ACK =1; 
				I2C1_CR2_ITBUFEN=1; 
 				}
			else
				{
				I2C1_CR1_ACK =0; 
				I2C1_CR2_ITBUFEN=1; 
				} 
			} 
		} 
	if(SR1&0x0004)//BTF 
		{ 
 		if(I2C_RunFlag==0x03) 
			{	
			I2C1_CR1_START=1; 
			I2C_RunFlag=0x02; 
			while(I2C1->SR1&0x0004);//没办法硬件有BUG只能强行等待否则会多次进入中断	
			} 
		else 
			{  
			I2C1_CR2_ITEVTEN=0;
			I2C1_CR2_ITERREN=0;
 			I2C1_CR1_STOP=1;
			while(I2C1->SR1&0x0004);
			} 
		} 
	if(I2C1_CR2_ITBUFEN>0) 
		{ 
		if(SR1&0x0080)//TXE 
			{       
 			I2C1->DR=*I2C_DataPoint++; 
 			I2C_ByteNum--; 
 			if(I2C_ByteNum<=0) 
				{ 
				I2C1_CR2_ITBUFEN=0; 
				} 
			} 
		if(SR1&0x0040)//RXNE 
   			{       
 			*I2C_DataPoint++=I2C1->DR;
 			I2C_ByteNum--;
 			if(I2C_ByteNum<=1)
  				{ 
				if(I2C_ByteNum>0) 
 					{ 
					I2C1_CR1_ACK =0; 
					I2C1_CR1_STOP=1;  
					} 
				else 
 					{ 
					I2C1_CR2_ITBUFEN=0; 
					I2C1_CR2_ITEVTEN=0; 
					I2C1_CR2_ITERREN=0;   
 					} 
				} 
			} 
		} 
} 

void I2C1_ER_IRQHandler(void) 
{ 
	u16 SR1,SR2;	
	SR1=I2C1->SR1; 
	if(SR1&0x0400)//AF 
		{ 
		I2C1->SR1&=~0x0400; 
		I2C1_CR2_ITEVTEN=0; 
		I2C1_CR2_ITERREN=0;  
		I2C1_CR2_ITBUFEN=0; 
		I2C1_CR1_STOP=1; 
		} 
} 

//
