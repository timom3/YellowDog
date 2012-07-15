/*---------------------------------
-  functions.h
-----------------------------------*/
#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*---------------------------
 define
 ---------------------------*/
#define KALMAN_DATA_TYPE int32_t
#define KALMAN_P 6	//状态
#define KALMAN_M 6	//观测
#define MATRIX_DATA_TYPE int32_t
#define MATRIX_AMOUNT (MATRIX_ORDER*MATRIX_ORDER)
#define MATRIX_ORDER	6
#define BIT32_ADD_OFFSET 1	//keil自动将其地址+4
#define KALMAN_Q 250
#define KALMAN_A_C 250 
#define KALMAN_G_C 5
#define KALMAN_Q_FLOAT 0.0000005
#define KALMAN_A_C_FLOAT 80000.0
#define KALMAN_G_C_FLOAT 80000.0
#define KALMAN_G_H 130992 //114285.0	//114285.0 正常(角速度) 114.285就变成角度(积分了以后)了
#define GYRO_DIGIT 0.00875
#define ANGtoRAD 0.0174533
#define RADtoANG 57.2957795

/*---------------------------------
-----------------------------------*/
typedef struct
{
	KALMAN_DATA_TYPE K[KALMAN_P*KALMAN_M];		//卡尔曼增益
	KALMAN_DATA_TYPE S[KALMAN_P];				//滤波后的估值
	KALMAN_DATA_TYPE x[KALMAN_P];				//观测向量
	KALMAN_DATA_TYPE M[KALMAN_P*KALMAN_P];		//最小MSE矩阵
	KALMAN_DATA_TYPE M_pre[KALMAN_P*KALMAN_P];	//最小预测MSE矩阵
	KALMAN_DATA_TYPE H[KALMAN_M*KALMAN_P];		//观测矩阵				??
	KALMAN_DATA_TYPE C[KALMAN_M*KALMAN_M];		//观测噪声的协方差阵	??
	KALMAN_DATA_TYPE A[KALMAN_P*KALMAN_P];		//状态转移矩阵			??
	KALMAN_DATA_TYPE Q[KALMAN_P*KALMAN_P];		//模型噪声的协方差阵	??		
	
	KALMAN_DATA_TYPE MM_temp[KALMAN_M*KALMAN_M];
	KALMAN_DATA_TYPE PM_temp[KALMAN_P*KALMAN_M];
	KALMAN_DATA_TYPE MP_temp[KALMAN_P*KALMAN_M];
	KALMAN_DATA_TYPE PP_temp[KALMAN_P*KALMAN_P];
}kalman_filter_TypeDef;

typedef struct
{
	float K[KALMAN_P];		//卡尔曼增益
	float S[KALMAN_P];				//滤波后的估值
	float S_pre[KALMAN_P];
	float x[KALMAN_P];				//观测向量
	float M[KALMAN_P];		//最小MSE矩阵
	float M_pre[KALMAN_P];	//最小预测MSE矩阵
	float H[KALMAN_P];		//观测矩阵				??
	float C[KALMAN_P];		//观测噪声的协方差阵	??
	float A[KALMAN_P];		//状态转移矩阵			??
	float Q[KALMAN_P];		//模型噪声的协方差阵	??		
	
//	float MM_temp[KALMAN_M*KALMAN_M];
	float PM_temp[KALMAN_P];
//	float MP_temp[KALMAN_M*KALMAN_P];
	float PP_temp[KALMAN_P];	
}kalmanFilterFloat_TypeDef;

typedef struct
{
	float t;
	float x,y,z;
	float normalizeDenominator;
	float dthet2;
	float dTHET[16];

	float pitch;
	float roll;
	float yaw;
	
}quaternion_TypeDef;

typedef struct
{
	MATRIX_DATA_TYPE A[MATRIX_AMOUNT];
	MATRIX_DATA_TYPE B[MATRIX_AMOUNT];
	MATRIX_DATA_TYPE C[MATRIX_AMOUNT];	// C=A?B
	uint8_t row;
	uint8_t colune;
}matrix_TypeDef;

/*-----------------------------------------
- extern data
------------------------------------------*/
extern kalman_filter_TypeDef k_filter;
extern kalmanFilterFloat_TypeDef kFFloat;
extern quaternion_TypeDef quat;

/*-----------------------------------------
------------------------------------------*/
void kalman_filter(void);
void kalman_init(void);
void kalman_gain(void);
void amend(void);
void minimum_MSE(void);
void minimum_prediction_MSE(void);
void matrix_sum(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C, uint8_t row, uint8_t column);
void matrix_mul(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C, uint8_t row, uint8_t column);
void matrix_inv(int32_t *matrix_A, int32_t *matrix_C);	//, uint8_t order
void int32ToChar(int32_t *data32, uint8_t *data8, int16_t data32size, int16_t data8Offset);
void kalmanFilterFloat(void);
void kalmanFilterFloatInit(void);
void floatToChar(float *data32, uint8_t *data8, int16_t data32size, int16_t data8Offset);
void quaternion(void);
void quaternionInit(void);


#endif
