/*------------------------------
test.c
	����
-------------------------------*/
#include "test.h"


/*-----------------------------
����������� (order=6)
-����: matrix_A
-���: matrix_C
-------------------------------*/
uint8_t matrix_test(void)
{
//#ifdef MATRIX_TEST	     
	MATRIX_DATA_TYPE const volatile matrix_test_A[36]={2,1,0,0,0,0,\
												1,2,0,0,0,0,\
												3,0,2,0,0,0,\
												0,0,0,2,0,0,\
												0,0,0,0,2,0,\
												0,0,0,0,0,2};
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
	
	matrix_sum(matrix_A, matrix_B, matrix_C, MATRIX_ORDER, MATRIX_ORDER);	//����ӷ�����
	printf("\nmatrix sum is: \n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;
		}
	matrix_C=(int32_t *)(&matrix_test_C[0]);
	
	matrix_mul(matrix_A, matrix_B, matrix_C, MATRIX_ORDER, MATRIX_ORDER);	//����˷�����
	printf("\nmatrix multiply is:\n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;		
		}
	matrix_C=(int32_t *)(&matrix_test_C[0]);
	
	matrix_inv(matrix_A,matrix_C);		//���������
	printf("\nmatrix invers is:\n");
	for(count=0;count<MATRIX_ORDER*MATRIX_ORDER; count++)
		{
		printf("%d ",*matrix_C);
		matrix_C++;		
		}

	while(1);
//#endif
}
