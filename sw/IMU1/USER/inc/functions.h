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
#define KALMAN_P 6	//״̬
#define KALMAN_M 6	//�۲�
#define MATRIX_DATA_TYPE int32_t
#define MATRIX_AMOUNT (MATRIX_ORDER*MATRIX_ORDER)
#define MATRIX_ORDER	6
#define BIT32_ADD_OFFSET 1	//keil�Զ������ַ+4
#define KALMAN_Q 250
#define KALMAN_A_C 250 
#define KALMAN_G_C 5
#define KALMAN_Q_FLOAT 0.0000005
#define KALMAN_A_C_FLOAT 80000.0
#define KALMAN_G_C_FLOAT 80000.0
#define KALMAN_G_H 130992 //114285.0	//114285.0 ����(���ٶ�) 114.285�ͱ�ɽǶ�(�������Ժ�)��
#define GYRO_DIGIT 0.00875
#define ANGtoRAD 0.0174533
#define RADtoANG 57.2957795

/*---------------------------------
-----------------------------------*/
typedef struct
{
	KALMAN_DATA_TYPE K[KALMAN_P*KALMAN_M];		//����������
	KALMAN_DATA_TYPE S[KALMAN_P];				//�˲���Ĺ�ֵ
	KALMAN_DATA_TYPE x[KALMAN_P];				//�۲�����
	KALMAN_DATA_TYPE M[KALMAN_P*KALMAN_P];		//��СMSE����
	KALMAN_DATA_TYPE M_pre[KALMAN_P*KALMAN_P];	//��СԤ��MSE����
	KALMAN_DATA_TYPE H[KALMAN_M*KALMAN_P];		//�۲����				??
	KALMAN_DATA_TYPE C[KALMAN_M*KALMAN_M];		//�۲�������Э������	??
	KALMAN_DATA_TYPE A[KALMAN_P*KALMAN_P];		//״̬ת�ƾ���			??
	KALMAN_DATA_TYPE Q[KALMAN_P*KALMAN_P];		//ģ��������Э������	??		
	
	KALMAN_DATA_TYPE MM_temp[KALMAN_M*KALMAN_M];
	KALMAN_DATA_TYPE PM_temp[KALMAN_P*KALMAN_M];
	KALMAN_DATA_TYPE MP_temp[KALMAN_P*KALMAN_M];
	KALMAN_DATA_TYPE PP_temp[KALMAN_P*KALMAN_P];
}kalman_filter_TypeDef;

typedef struct
{
	float K[KALMAN_P];		//����������
	float S[KALMAN_P];				//�˲���Ĺ�ֵ
	float S_pre[KALMAN_P];
	float x[KALMAN_P];				//�۲�����
	float M[KALMAN_P];		//��СMSE����
	float M_pre[KALMAN_P];	//��СԤ��MSE����
	float H[KALMAN_P];		//�۲����				??
	float C[KALMAN_P];		//�۲�������Э������	??
	float A[KALMAN_P];		//״̬ת�ƾ���			??
	float Q[KALMAN_P];		//ģ��������Э������	??		
	
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
