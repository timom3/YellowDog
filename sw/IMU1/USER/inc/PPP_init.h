/***********************************
-  PPP_init.h
************************************/
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdlib.h>

/***********************************
- define
************************************/
#define I2C2_Speed 400000
#define LED_OFF (GPIO_SetBits(GPIOB,GPIO_Pin_5))
#define LED_ON  (GPIO_ResetBits(GPIOB,GPIO_Pin_5))
#define LED_TOGGLE (GPIOB->ODR^=GPIO_Pin_5)
#define I2C2_DR_Address (I2C2_BASE+0x10)	// I2C2起始地址 0x40005800  I2C_DR偏移量0x10
#define SENSOR_I2C_BUS I2C2
#define SYSTICK_TICK 72000		//1ms

//sensor_state.I2C_run_flag 识别宏定义
#define I2C_FREE_MASK 0x08//第3位为0时 I2C空闲
#define ACCER_READING 0x80
#define GYRO_READING 0x40
#define MEGN_READING 0x20

#define ACCER_DATA_BASE (&sensor_data.ACCER_OUT_X_L)
#define GYRO_DATA_BASE (&sensor_data.GYRO_OUT_X_L)
//#define MEGN_DATA_BASE (&sensor_data.MEGN_OUT_X_L)


// MEMS传感器内部寄存器地址
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
#define OUT_X_L_G 0x28 			//0xA8
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
#define SEND_SUB  0x03
#define RECEIVE_DATA 0x02



/***********************************
- struct
************************************/
typedef struct
{
	uint8_t sensor_address;
	__IO uint8_t I2C_run_flag;/*  (MSB)AGMR xxxx(LSB)
									 A-加速度计状态表示, 1为正在执行 0为空闲;
									 G-陀螺仪
									 M-磁传感器
									 R-运行标志,0为空闲,1为运行
									 x-对应SUB偏移量,若x全部为0 则I2C空闲*/
	uint8_t SUB;
	__IO uint8_t receive_data;
	uint8_t data_direction;
	FlagStatus SUB_Transmitted;
	FlagStatus I2C_bus_busy;	//DMA从I2C的DR中读出数据后清除
	FlagStatus data_receive_finish;
}sensor_state_TypeDef;

typedef struct
{
	uint8_t ACCER_OUT_X_L;	uint8_t ACCER_OUT_X_H;
	uint8_t ACCER_OUT_Y_L;	uint8_t ACCER_OUT_Y_H;
	uint8_t ACCER_OUT_Z_L;	uint8_t ACCER_OUT_Z_H;
	
	uint8_t GYRO_OUT_X_L;	uint8_t GYRO_OUT_X_H;
	uint8_t GYRO_OUT_Y_L;	uint8_t GYRO_OUT_Y_H;
	uint8_t GYRO_OUT_Z_L;	uint8_t GYRO_OUT_Z_H;

	int16_t ACCER_X;	int16_t ACCER_Y;	int16_t ACCER_Z;
	int16_t GYRO_X;		int16_t GYRO_Y;		int16_t GYRO_Z;
}sensor_data_TypeDef;


/***********************************
- extern data
************************************/
extern sensor_data_TypeDef sensor_data;
extern sensor_state_TypeDef sensor_state;
extern uint16_t test_state;

/***********************************
- Function declarations
************************************/
void RCC_Configuration(void);
void USART1_Configuration(void);
void LED_Configuration(void);
void SENSOR_I2C_BUS_Configuration(void);
void SENSOR_I2C_BUS_DMA_Configuration(void);
void SYSTICK_Configuration(void);
ErrorStatus sensor_state_assign(uint8_t sensor_address, uint8_t SUB);
void SENSOR_I2C_BUS_EV_ISR(void);
void SENSOR_I2C_BUS_ERR_ISR(void);
void SYSTICK_ISR(void);
void SENSOR_EXTI_Configuration(void);









