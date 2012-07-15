/*------------------------------
test.h
	≤‚ ‘
-------------------------------*/
#ifndef TEST_H
#define TEST_H

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "PPP_init.h"
#include "functions.h"

//≤‚ ‘π¶ƒ‹—°‘Ò
//#define MATRIX_TEST


/*
extern MATRIX_DATA_TYPE matrix_test_A[];
extern MATRIX_DATA_TYPE matrix_test_B[];
extern MATRIX_DATA_TYPE matrix_test_C[];*/


extern uint8_t matrix_test(void);
uint8_t kalman_test(void);
void printfMatrix(MATRIX_DATA_TYPE *addr, int size);

#endif

