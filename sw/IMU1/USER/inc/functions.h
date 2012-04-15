/*---------------------------------
-  functions.h
-----------------------------------*/
#include "stm32f10x_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*---------------------------
 define
 ---------------------------*/
#define MATRIX_DATA_TYPE int32_t
#define MATRIX_AMOUNT (MATRIX_ORDER*MATRIX_ORDER)
#define MATRIX_ORDER	6
#define BIT32_ADD_OFFSET 1	//keil自动将其地址+4


typedef struct
{
	MATRIX_DATA_TYPE A[MATRIX_AMOUNT];
	MATRIX_DATA_TYPE B[MATRIX_AMOUNT];
	MATRIX_DATA_TYPE C[MATRIX_AMOUNT];	// C=A?B
	uint8_t row;
	uint8_t colune;
}matrix_TypeDef;

void matrix_sum(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C, uint8_t row, uint8_t column);
void matrix_mul(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C, uint8_t row, uint8_t column);
void matrix_inv(int32_t *matrix_A, int32_t *matrix_C);	//, uint8_t order

