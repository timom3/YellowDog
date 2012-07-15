#define WHO_AM_I 0x0F 
#define CTRL_REG1_A 0x20 
#define CTRL_REG2_A 0x21 
#define CTRL_REG3_A 0x22 
#define CTRL_REG4_A 0x23 
#define CTRL_REG5_A 0x24 
#define CTRL_REG1_G 0x20 
#define CTRL_REG2_G 0x21 
#define CTRL_REG3_G 0x22 
#define CTRL_REG4_G 0x23 
#define CTRL_REG5_G 0x24 
#define REFERENCE 0x25 
#define OUT_TEMP 0x26 
#define STATUS_REG 0x27 
#define OUT_X_L_A 0x28 
#define OUT_X_H_A 0x29 
#define OUT_Y_L_A 0x2A 
#define OUT_Y_H_A 0x2B 
#define OUT_Z_L_A 0x2C 
#define OUT_Z_H_A 0x2D 
//#define OUT_X_L_G 0x28 
#define OUT_X_H_G 0x29 
#define OUT_Y_L_G 0x2A 
#define OUT_Y_H_G 0x2B 
#define OUT_Z_L_G 0x2C 
#define OUT_Z_H_G 0x2D 
#define FIFO_CTRL_REG 0x2E 
#define FIFO_SRC_REG 0x2F 
#define INT1_CFG 0x30 
#define INT1_SRC 0x31 
#define INT1_TSH_XH 0x32 
#define INT1_TSH_XL 0x33 
#define INT1_TSH_YH 0x34 
#define INT1_TSH_YL 0x35 
#define INT1_TSH_ZH 0x36 
#define INT1_TSH_ZL 0x37 
#define INT1_DURATION 0x38 
#define GYRO_address 0xD0
#define ACCER_address 0x30
#define MEGN_address 0x3C



void I2C_Configuration(void) 
{ 
	I2C_InitTypeDef I2C_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	// I2C Periph clock enable 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);   
 
	// GPIO Periph clock enable 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
	I2C_DeInit(I2C2); 

	/* I2C configuration */ 
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C; 
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;    
	I2C_InitStructure.I2C_OwnAddress1 = 0xDD; //0xD0 gyro µØÖ·
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable; 
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; 
	I2C_InitStructure.I2C_ClockSpeed = 200000; 

	// Configure I2C2 SCL as alternate function Open-drain output 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    
	// Configure I2C2 SDA as alternate function Open-drain output  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C Peripheral Enable */ 
	I2C_Cmd(I2C2, ENABLE); 

	/* Apply I2C configuration after enabling it */ 
	I2C_Init(I2C2, &I2C_InitStructure); 
} 

void sensor_write(uint8_t sensor_addr, uint8_t addr, uint8_t data)//write a byte to sensor
{ 
	int dummy; 

	/* Send STRAT condition */ 
	I2C_GenerateSTART(I2C2, ENABLE); 

	/* Test on EV5 and clear it */ 
	while(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS); 

	/* Send EEPROM address for write */ 
	dummy=I2C2->SR1; 
	sensor_addr=sensor_addr&0xfe;
	I2C_Send7bitAddress(I2C2, sensor_addr, I2C_Direction_Transmitter); 

	/* Test on EV6 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));       

	/* Send the EEPROM's internal address to write to */ 
	I2C_SendData(I2C2, addr); 

	/* Test on EV8 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); 

	/* Send the byte to be written */ 
	I2C_SendData(I2C2, data);    

	/* Test on EV8 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); 

	/* Send STOP condition */ 
	I2C_GenerateSTOP(I2C2, ENABLE); 
} 

uint8_t sensor_read(uint8_t sensor_address, uint8_t SUB)//read a byte from sensor
{ 
	uint8_t data;  

	/* Send START condition */ 
	I2C_GenerateSTART(I2C2, ENABLE); 

	/* Test on EV5 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)); 

	/* In the case of a single data transfer disable ACK before reading the data */ 
    I2C_AcknowledgeConfig(I2C2, DISABLE); 

	/* Send EEPROM address for write */ 
	sensor_address=sensor_address|0x01;
	I2C_Send7bitAddress(I2C2, sensor_address, I2C_Direction_Transmitter); //0xD2

	/* Test on EV6 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 

	/* Send the EEPROM's internal address to write to */ 
	I2C_SendData(I2C2, SUB);  

	/* Test on EV8 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); 

	/* Send STRAT condition a second time */ 
	I2C_GenerateSTART(I2C2, ENABLE); 

	/* Test on EV5 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)); 

	/* Send EEPROM address for read */ 
	I2C_Send7bitAddress(I2C2, sensor_address, I2C_Direction_Receiver); 

	/* Test on EV6 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 

	/* Send STOP Condition */ 
    I2C_GenerateSTOP(I2C2, ENABLE); 

	/* Test on EV7 and clear it */ 
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));    

	/* Read a byte from the EEPROM */ 
    data = I2C_ReceiveData(I2C2);   
	
	return data;   
} 

void GYRO_Init()
{ 
	I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_BUF|I2C_IT_EVT|I2C_IT_ERR, DISABLE);
	
/**/sensor_write(ACCER_address, CTRL_REG1_A, 0x3F);//0x2F 
	sensor_write(ACCER_address, CTRL_REG2_A, 0x00); 
	sensor_write(ACCER_address, CTRL_REG3_A, 0x10); //0x1A
	sensor_write(ACCER_address, CTRL_REG4_A, 0x00); 
	sensor_write(ACCER_address, CTRL_REG5_A, 0x00);
	
//	SENSOR_EXTI_Configuration();
	
	sensor_write(GYRO_address, CTRL_REG1_G, 0xEF); //0xEF
	sensor_write(GYRO_address, CTRL_REG2_G, 0x00);
	sensor_write(GYRO_address, CTRL_REG3_G, 0x08);
	sensor_write(GYRO_address, CTRL_REG4_G, 0x00);
	sensor_write(GYRO_address, CTRL_REG5_G, 0x00);

//	printf("\r\nGYRO_init DONE!\r\n");
	
	I2C_ITConfig(SENSOR_I2C_BUS, I2C_IT_EVT|I2C_IT_ERR, ENABLE);		
} 

void GYRO_Test()
{ 
	int value1=0,value2=0,value3=0; 
	value1= sensor_read(GYRO_address, OUT_X_H_G)*0x100 + sensor_read(GYRO_address, OUT_X_L_G); 

	if(value1 > 0x7FFF) 
		value1=value1-0x10000; 

	value2= sensor_read(GYRO_address, OUT_Y_H_G)*0x100 + sensor_read(GYRO_address, OUT_Y_L_G); 

	if(value2 > 0x7FFF) 
		value2=value2-0x10000; 

	value3= sensor_read(GYRO_address, OUT_Z_H_G)*0x100 + sensor_read(GYRO_address, OUT_Z_L_G); 

	if(value3 > 0x7FFF) 
		value3=value3-0x10000; 

	printf("gyro :%d %d %d   ",value1,value2,value3); 

} 

void ACCER_Test()
{ 
	int value1=0,value2=0,value3=0; 
	sensor_write(ACCER_address, CTRL_REG1_A, 0x3F);
	
	value1= sensor_read(ACCER_address, OUT_X_H_A)*0x100 + sensor_read(ACCER_address, OUT_X_L_A); 
	
	if(value1 > 0x7FFF) 
		value1=value1-0x10000; 

	value2= sensor_read(ACCER_address, OUT_Y_H_A)*0x100 + sensor_read(ACCER_address, OUT_Y_L_A); 

	if(value2 > 0x7FFF) 
		value2=value2-0x10000; 

	value3= sensor_read(ACCER_address, OUT_Z_H_A)*0x100 + sensor_read(ACCER_address, OUT_Z_L_A);  

	if(value3 > 0x7FFF) 
		value3=value3-0x10000; 
/*
	STATUS_RES_A=sensor_read(ACCER_address,0x20);
	printf("CTRL1_RES_A: 0x%x  ", STATUS_RES_A);
	STATUS_RES_A=sensor_read(ACCER_address,0x21);
	printf("CTRL2_RES_A: 0x%x  ", STATUS_RES_A);
	STATUS_RES_A=sensor_read(ACCER_address,0x22);
	printf("CTRL3_RES_A: 0x%x  ", STATUS_RES_A);
	STATUS_RES_A=sensor_read(ACCER_address,0x23);
	printf("CTRL4_RES_A: 0x%x  ", STATUS_RES_A);
	STATUS_RES_A=sensor_read(ACCER_address,0x24);
	printf("CTRL5_RES_A: 0x%x  ", STATUS_RES_A);
*/	
	printf("accer:%d %d %d\n",value1,value2,value3);
//	printf("accer:0x%x 0x%x 0x%x\n",value1,value2,value3);


} 

