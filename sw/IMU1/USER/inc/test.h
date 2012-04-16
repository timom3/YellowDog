/*------------------------------
test.h
	≤‚ ‘
-------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "functions.h"



MATRIX_DATA_TYPE matrix_test_A[36]={0};
MATRIX_DATA_TYPE matrix_test_B[36]={0};
MATRIX_DATA_TYPE matrix_test_C[36]={0};


#ifdef MATRIX_TEST
uint8_t matrix_test(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C);
#endif


