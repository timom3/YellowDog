/***********************************
-  functions.c
-		卡尔曼
-		四元数
************************************/
#include "functions.h"

matrix_TypeDef matrix;

/*-------------------
- 卡尔曼滤波
---------------------
void kalman_filter()
{
	minimum_prediction_MSE();	//-1.最小预测MSE矩阵
	kalman_gain();				//-2.卡尔曼增益矩阵
	minimum_MSE();				//-3.最小MSE矩阵
	predict();					//-4.预测
	amend();					//-5.修正
}*/


/*-------------------
- 最小预测MSE矩阵
---------------------
void minimum_prediction_MSE()
{
	
}*/

/*-------------------
- 卡尔曼增益矩阵
---------------------
void kalman_gain()
{}*/

/*-------------------
- 最小MSE矩阵
---------------------
void minimum_MSE()
{}*/

/*-------------------
- 预测
---------------------
void predict()
{}*/

/*-------------------
- 修正
---------------------
void amend()
{}*/

/*-------------------
- 矩阵相加
---------------------*/
void matrix_sum(MATRIX_DATA_TYPE* matrix_A, MATRIX_DATA_TYPE* matrix_B, MATRIX_DATA_TYPE* matrix_C, uint8_t row, uint8_t column)
{
	int8_t count;
	for(count=0; count<row*column; count++)
		{
		(*matrix_C) = (*matrix_A)+(*matrix_B);
		matrix_A+=BIT32_ADD_OFFSET;
		matrix_B+=BIT32_ADD_OFFSET;
		matrix_C+=BIT32_ADD_OFFSET;
		}
}
/*-------------------
- 矩阵相乘
---------------------*/
void matrix_mul(MATRIX_DATA_TYPE *matrix_A, MATRIX_DATA_TYPE *matrix_B, MATRIX_DATA_TYPE *matrix_C, uint8_t row, uint8_t column)
{
	uint8_t row_temp=0, column_temp=0;	//指示当前计算的行,列
	uint8_t a=0, i=0;	//a:当前计算到的元素 列的计数
	uint8_t temp;
	MATRIX_DATA_TYPE *matrix_A_store=matrix_A;
	MATRIX_DATA_TYPE *matrix_B_store=matrix_B;
	MATRIX_DATA_TYPE *add;
	MATRIX_DATA_TYPE a_temp,b_temp,c_temp;

	for(i=0;i<row*column;i++)	//将输出的矩阵清零
		{
		*matrix_C=0;
		matrix_C+=BIT32_ADD_OFFSET;
		}
	matrix_C-=(row*column*BIT32_ADD_OFFSET);
	
	for(i=0;i<row*column;i++)
		{
		c_temp=0;		  
		matrix_A=matrix_A_store;
		matrix_A+=(((i/column)*column)*BIT32_ADD_OFFSET);
		matrix_B=matrix_B_store;
		matrix_B+=((i%row)*BIT32_ADD_OFFSET);
		for(temp=0;temp<column;temp++)	//计算C[i]
			{
			add=(matrix_A+column_temp*BIT32_ADD_OFFSET);
			a_temp=(*add);
			add=(matrix_B+row_temp*BIT32_ADD_OFFSET);
			b_temp=(*add);
			c_temp+=a_temp*b_temp;
//			c_temp+=(*(matrix_A+column_temp*BIT32_ADD_OFFSET))*(*(matrix_B+row_temp*BIT32_ADD_OFFSET));
//			(*matrix_C)+=(*(matrix_A+column_temp*BIT32_ADD_OFFSET))*(*(matrix_B+row_temp*BIT32_ADD_OFFSET));
			column_temp++;
			row_temp+=column;
			}
		row_temp=0;
		column_temp=0;
		(*matrix_C)=c_temp;
		matrix_C+=BIT32_ADD_OFFSET;	//计算C[i+1]
		}	
}
/*-------------------
- 矩阵求逆 (定点数)
  输入 方阵是6*6 矩阵A首地址
  输出 矩阵C
---------------------*/
void matrix_inv(int32_t *matrix_A, int32_t *matrix_C)	//, uint8_t order
{
	uint8_t row,column;
	int8_t k,l,u,v;
	uint8_t rows[MATRIX_AMOUNT],columns[MATRIX_AMOUNT];
	MATRIX_DATA_TYPE d,p;

	for(k=0;k<=MATRIX_ORDER-1;k++)
		{
		*matrix_C = *matrix_A;
		matrix_C += BIT32_ADD_OFFSET;
		matrix_A += BIT32_ADD_OFFSET;
		}
	matrix_C -= 36*BIT32_ADD_OFFSET;
	matrix_A -= 36*BIT32_ADD_OFFSET;

	for(k=0;k<=MATRIX_ORDER-1;k++)
		{
		d=0;
		for(row=k;row<=MATRIX_ORDER-1;row++)
			{
			for(column=k;column<=MATRIX_ORDER-1;column++)
				{
				l=row*MATRIX_ORDER+column;
				p=abs(*(matrix_A+l*BIT32_ADD_OFFSET));
				if(p>d)
					{
					d=p;
					rows[k]=row;
					columns[k]=column;
					}
				}
			}
	/*	if((d/100)<1)	//上面搜索的元素均很小,矩阵不能求逆
			{
			}*/
		if(rows[k]!=k)
			{
			for(column=0;column<=MATRIX_ORDER-1;column++)
				{
				u=k*MATRIX_ORDER+column;
				v=rows[k]*MATRIX_ORDER+column;
				p=*(matrix_C+u*BIT32_ADD_OFFSET);
				*(matrix_C+u*BIT32_ADD_OFFSET) = *(matrix_C+v*BIT32_ADD_OFFSET);
				*(matrix_C+v*BIT32_ADD_OFFSET) = p;
				}
			}
		if(columns[k]!=k)
			{
			for(row=0;row<=MATRIX_ORDER-1;row++)
				{
				u=row*MATRIX_ORDER+k;
				v=row*MATRIX_ORDER+columns[k];
				p=*(matrix_C+u*BIT32_ADD_OFFSET);
				*(matrix_C+u*BIT32_ADD_OFFSET) = *(matrix_C+v*BIT32_ADD_OFFSET);	
				*(matrix_C+v*BIT32_ADD_OFFSET) = p;
				}
			}
		l=k*MATRIX_ORDER+k;		
		//a[kk]=1/a[kk]
		*(matrix_C+l*BIT32_ADD_OFFSET) = (1000*1000)/(*matrix_C+l*BIT32_ADD_OFFSET);	//, 1=>1000
		for(column=0;column<=MATRIX_ORDER-1;column++)
			{
			if(column!=k)
				{
				u=k*MATRIX_ORDER+column;
				//a[kj]=a[kj]a[kk]
				*(matrix_C+u*BIT32_ADD_OFFSET)=((*(matrix_C+u*BIT32_ADD_OFFSET))*(*(matrix_C+l*BIT32_ADD_OFFSET)))/1000;	
				}
			}
		for(row=0;row<=MATRIX_ORDER;row++)
			{
			if(row!=k)
				{
				for(column=0;column<=MATRIX_ORDER-1;column++)
					{
					if(column!=k)
						{
						u=row*MATRIX_ORDER+column;
						//a[ij]=a[ij]-a[ik]a[kj]
						*(matrix_C+u*BIT32_ADD_OFFSET)=(*(matrix_C+u*BIT32_ADD_OFFSET))-\
						(((*(matrix_C+(row*MATRIX_ORDER+k)*BIT32_ADD_OFFSET))*(*(matrix_C+(k*MATRIX_ORDER+column)*BIT32_ADD_OFFSET))))/1000;
						}
					}
				}
			}
		for(row=0;row<=MATRIX_ORDER-1;row++)
			{
			if(row!=k)
				{
				u=row*MATRIX_ORDER+k;
				//a[ik]=-a[ik]a[kk]
				*(matrix_C+u*BIT32_ADD_OFFSET)=-(*(matrix_C+u*BIT32_ADD_OFFSET))*(*(matrix_C+l*BIT32_ADD_OFFSET));
				}
			}
		}
	for(k=MATRIX_ORDER-1;k>=0;k--)
		{
		if(columns[k]!=k)
			{
			for(column=0;column<=MATRIX_ORDER-1;column++)
				{
				u=k*MATRIX_ORDER+column;
				v=columns[k]*MATRIX_ORDER+column;
				p=*(matrix_C+u*BIT32_ADD_OFFSET);
				*(matrix_C+u*BIT32_ADD_OFFSET)=*(matrix_C+v*BIT32_ADD_OFFSET);
				*(matrix_C+v*BIT32_ADD_OFFSET)=p;
				}
			}
		if(rows[k]!=k)
			{
			for(row=0;row<MATRIX_ORDER-1;row++)
				{
				u=row*MATRIX_ORDER+k;
				v=row*MATRIX_ORDER+rows[k];
				p=*(matrix_C+u*BIT32_ADD_OFFSET);
				*(matrix_C+u*BIT32_ADD_OFFSET)=*(matrix_C+v*BIT32_ADD_OFFSET);
				*(matrix_C+v*BIT32_ADD_OFFSET)=p;
				}
			}
		}
}

