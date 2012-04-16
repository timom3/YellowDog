/*------------------------------
test.c
	≤‚ ‘
-------------------------------*/
#include "test.h"


/*-----------------------------
æÿ’Û‘ÀÀ„≤‚ ‘ (order=6)
- ‰»Î: matrix_A
- ‰≥ˆ: matrix_C
-------------------------------*/
uint8_t matrix_test(void)
{
//#ifdef MATRIX_TEST	     
	MATRIX_DATA_TYPE const volatile matrix_test_A[36]={
	2000,3000,0,0,0,0,\
	1000,4000,0,9000,7000,6000,\
	3000,7000,6000,8000,0,0,\
	0,1000,2000,1000,2000,0,\
	0,0,2000,1000,3000,1000,\
	0,0,2000,4000,2000,3000	
	};
	MATRIX_DATA_TYPE const volatile matrix_test_B[36]={2,1,3,0,0,0,\
												1,2,0,0,0,0,\
												0,0,2,0,0,0,\
												0,0,0,2,0,0,\
												0,0,0,0,2,0,\
												0,0,0,0,0,2	};
	MATRIX_DATA_TYPE volatile matrix_test_C[36]={0};
	MATRIX_DATA_TYPE *matrix_A;
	MATRIX_DATA_TYPE *matrix_B;
	MATRIX_DATA_TYPE *matrix_C;

	uint8_t count;
	
	matrix_A=(MATRIX_DATA_TYPE *)(&matrix_test_A[0]);
	matrix_B=(MATRIX_DATA_TYPE *)(&matrix_test_B[0]);
	matrix_C=(MATRIX_DATA_TYPE *)(&matrix_test_C[0]);
	
	matrix_sum(matrix_A, matrix_B, matrix_C, MATRIX_ORDER, MATRIX_ORDER);	//æÿ’Ûº”∑®≤‚ ‘
	printf("\r\nmatrix sum is: \r\n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;
		}
	matrix_C=(int32_t *)(&matrix_test_C[0]);
	
	matrix_mul(matrix_A, matrix_B, matrix_C, MATRIX_ORDER, MATRIX_ORDER);	//æÿ’Û≥À∑®≤‚ ‘
	printf("\nmatrix multiply is:\r\n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;		
		}
	matrix_C=(int32_t *)(&matrix_test_C[0]);
	
	matrix_inv(matrix_A,matrix_C);		//æÿ’ÛƒÊ≤‚ ‘
	printf("\r\nmatrix invers is:\r\n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;		
		}

	while(1);
//#endif
}
