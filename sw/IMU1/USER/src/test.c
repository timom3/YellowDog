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

/*-----------------------------
ø®∂˚¬¸≤‚ ‘
- ‰»Î: 
- ‰≥ˆ: 
-------------------------------*/
uint8_t kalman_test(void)
{
	int i;
	
//	kalman_init();
/*	k_filter.x[0]=sensor_data.ACCER_X;
	k_filter.x[1]=sensor_data.ACCER_Y;
	k_filter.x[2]=sensor_data.ACCER_Z;
	k_filter.x[3]=sensor_data.GYRO_X;
	k_filter.x[4]=sensor_data.GYRO_Y;
	k_filter.x[5]=sensor_data.GYRO_Z;

	kalman_filter();
//	int32ToChar(&k_filter.S[0], &UsartData.TxBuffer[0], KALMAN_P, USART_A_OFFSET);
// 	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	printf("S:");
	for(i=0;i<KALMAN_P;i++)
		{
		printf("%d ",k_filter.S[i]);
		}
	printf("\r\n");
	*/
	return 1;
}

/*-----------------------------
∏°µ„ø®∂˚¬¸≤‚ ‘
- ‰»Î: 
- ‰≥ˆ: 
-------------------------------*/
void kalmanFloatTest(void)
{
	int16_t i;
	
	kFFloat.x[0]=sensor_data.ACCER_X;
	kFFloat.x[1]=sensor_data.ACCER_Y;
	kFFloat.x[2]=sensor_data.ACCER_Z;
/*	kFFloat.x[0]=sensor_data.GYRO_X;
	kFFloat.x[1]=sensor_data.GYRO_Y;
	kFFloat.x[2]=sensor_data.GYRO_Z;*/
	kFFloat.x[3] =sensor_data.GYRO_X;
	kFFloat.x[4] =sensor_data.GYRO_Y;
	kFFloat.x[5] =sensor_data.GYRO_Z;

	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.S_pre[i]=kFFloat.S[i];
		}
	
	kalmanFilterFloat();
	
/*	printf("\r\nS:");
	for(i=0;i<KALMAN_P;i++)
		{
		printf("%8f ",kFFloat.S[i]);
		}*/
//	printf("\r\nx:%2f %2f %2f|",kFFloat.x[3],kFFloat.x[4],kFFloat.x[5]);
//	printf("\r\nS:%4f %4f %4f|",kFFloat.PP_temp[3],kFFloat.PP_temp[4],kFFloat.PP_temp[5]);
}

/*-----------------------------
¥Ú”°æÿ’Û∏˜∏ˆ‘™Àÿ
- ‰»Î: æÿ’Ûµÿ÷∑  ˝¡ø
- ‰≥ˆ: 
-------------------------------*/
void printfMatrix(MATRIX_DATA_TYPE *addr, int size)
{
	int i;
	for(i=0;i<size;i++)
		{
		printf("%d ", *(addr+i));
		}
//	printf("r\n");
}

/*-----------------------------
Àƒ‘™ ˝≤‚ ‘
- ‰»Î: 
- ‰≥ˆ: 
-------------------------------*/
void quaternionTest(void)
{
//	int i;
	
	quaternion();
	printf("\r\nQ:%6f %6f %6f|",quat.x,quat.y,quat.z);
//	printf(" pitch:%6f| roll: %6f| yaw:%6f|\r\n", quat.x*RADtoANG, quat.y*RADtoANG, quat.z*RADtoANG);
}

