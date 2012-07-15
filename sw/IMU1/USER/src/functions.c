/***********************************
-  functions.c
-		卡尔曼
-		四元数
************************************/
#include "functions.h"
#include "test.h"

kalman_filter_TypeDef k_filter;
kalmanFilterFloat_TypeDef kFFloat;
quaternion_TypeDef quat;
matrix_TypeDef matrix;

/*-------------------
- 卡尔曼滤波
---------------------*/
void kalman_filter(void)
{
	kalman_gain();				//-1.卡尔曼增益矩阵
	amend();					//-2.修正
	minimum_MSE();				//-3.最小MSE矩阵
	minimum_prediction_MSE();	//-4.最小预测MSE矩阵
}

/*-------------------
- 卡尔曼滤波器参数初始化
---------------------*/
void kalman_init(void)
{
	int i;
	for(i=0;i<KALMAN_P;i++)
		{
		k_filter.S[i]=10;
		}
	for(i=0;i<KALMAN_P*KALMAN_M;i++)
		{
		k_filter.A[i]=0;
		k_filter.Q[i]=0;
		k_filter.H[i]=0;
		k_filter.C[i]=0;
		}
	
	//初始化A
	k_filter.A[0]=1000; k_filter.A[7]=1000; k_filter.A[14]=1000;	//1.000 = 1000
	k_filter.A[21]=1000;k_filter.A[28]=1000;k_filter.A[35]=1000;

	//初始化Q
	//--假设Q为0
	k_filter.Q[0]=KALMAN_Q; k_filter.Q[7]=KALMAN_Q; k_filter.Q[14]=KALMAN_Q;	//1.000 = 1000
	k_filter.Q[21]=KALMAN_Q;k_filter.Q[28]=KALMAN_Q;k_filter.Q[35]=KALMAN_Q;
	
	//初始化H
	k_filter.H[0]=16967; k_filter.H[7]=16967; k_filter.H[14]=16967;
//	k_filter.H[21]=22857;k_filter.H[28]=22857;k_filter.H[35]=22857;
	k_filter.H[21]=11428;k_filter.H[28]=11428;k_filter.H[35]=11428;

	
	//初始化C
	//	k_filter.C[0]=7284;k_filter.C[7]=7284;k_filter.C[14]=7284;
	//	k_filter.C[21]=4784;k_filter.C[28]=4784;k_filter.C[35]=4784;
	k_filter.C[0]=KALMAN_A_C;k_filter.C[7]=KALMAN_A_C;k_filter.C[14]=KALMAN_A_C;
	k_filter.C[21]=KALMAN_G_C;k_filter.C[28]=KALMAN_G_C;k_filter.C[35]=KALMAN_G_C;

	
	//初始化M
	k_filter.M[0]=KALMAN_A_C;k_filter.M[7]=KALMAN_A_C;k_filter.M[14]=KALMAN_A_C;
	k_filter.M[21]=KALMAN_G_C;k_filter.M[28]=KALMAN_G_C;k_filter.M[35]=KALMAN_G_C;
//	k_filter.M_pre[0]=KALMAN_A_C;k_filter.M_pre[7]=KALMAN_A_C;k_filter.M_pre[14]=KALMAN_A_C;
//	k_filter.M_pre[21]=KALMAN_G_C;k_filter.M_pre[28]=KALMAN_G_C;k_filter.M_pre[35]=KALMAN_G_C;

	
}

/*-------------------
- 卡尔曼增益矩阵
---------------------*/
void kalman_gain(void)
{
	uint8_t i,j,count,count1;
	KALMAN_DATA_TYPE MM[KALMAN_M*KALMAN_M];
	
	//1a.k_filter.MM_temp=H*M_pre	[M*p]
	matrix_mul(&(k_filter.H[0]),&(k_filter.M_pre[0]),&(k_filter.MM_temp[0]),KALMAN_M,KALMAN_P);
	printf("\r\n1 a.H*M_pre:");		printfMatrix(&k_filter.MM_temp[0], KALMAN_M*KALMAN_P);
	
	//1b.K=k_filter.MM_temp*H(T)	[M*M]
	for(i=0;i<KALMAN_M;i++)
		{
		for(j=0;j<KALMAN_M;j++)
			{
			count=i*KALMAN_M+j;
			k_filter.K[count]=0;
			for(count1=0;count1<KALMAN_M;count1++)
				{
				k_filter.K[count]+=((k_filter.MM_temp[i*KALMAN_M+count1]*k_filter.H[j*KALMAN_M+count1])/1000);
				}
			}
		}
	printf("\r\n1 b:");		printfMatrix(&k_filter.K[0], KALMAN_P*KALMAN_M);

	//1c.MM=C+K	[M*M]
	matrix_sum(&(k_filter.C[0]),&(k_filter.K[0]),&(MM[0]),KALMAN_M,KALMAN_M);
	printf("\r\n1 c:");		printfMatrix(&MM[0], KALMAN_M*KALMAN_M);
	
	//1d.k_filter.MM_temp=k_filter.PM_temp(T)	[M*M]
	matrix_inv(&(MM[0]),&(k_filter.MM_temp[0]));
	printf("\r\n1 d.(C+M_pre)inv:");	printfMatrix(&k_filter.MM_temp[0], KALMAN_M*KALMAN_M);
	
	//1f.k_filter.PM_temp=M_pre*H(T)	[p*M]
	for(i=0;i<KALMAN_P;i++)
		{
		for(j=0;j<KALMAN_M;j++)
			{
			count=i*KALMAN_M+j;
			k_filter.PM_temp[count]=0;
			for(count1=0;count1<KALMAN_M;count1++)
				{
				k_filter.PM_temp[count]+=((k_filter.M_pre[i*KALMAN_P+count1]*k_filter.H[j*KALMAN_M+count1])/1000);
				}
			}
		}
	printf("\r\n1 f:");		printfMatrix(&k_filter.PM_temp[0], KALMAN_P*KALMAN_M);
	
	//K=k_filter.PM_temp*k_filter.MM_temp	[p*M]
	matrix_mul(&(k_filter.PM_temp[0]),&(k_filter.MM_temp[0]),&(k_filter.K[0]),KALMAN_P,KALMAN_M);
	printf("\r\n1-.K:");	printfMatrix(&k_filter.K[0], KALMAN_P*KALMAN_M);
}

/*-------------------
- 修正
---------------------*/
void amend()
{
	uint8_t i,count1;
	KALMAN_DATA_TYPE S_temp[KALMAN_P];
	KALMAN_DATA_TYPE S1_temp[KALMAN_P];

	//MP_temp=H*A	[M*p]
	//matrix_mul(&(k_filter.H[0]),&(k_filter.A[0]),&(k_filter.MP_temp[0]),KALMAN_M,KALMAN_P);

	//2b.S_temp=(-H*A*S)	[M]T
	for(i=0;i<KALMAN_M;i++)
		{
		S_temp[i]=0;
		for(count1=0;count1<KALMAN_P;count1++)
			{
			S_temp[i]-=((k_filter.H[i*KALMAN_P+count1]*k_filter.S[count1])/1000);
			}
		}
	printf("\r\n2 b.(-H*S):");	printfMatrix(&S_temp[0], KALMAN_P);
	
	//2c.S_temp=x-H*A*S	[M]T
	matrix_sum(&(k_filter.x[0]),&(S_temp[0]),&(S_temp[0]),1,KALMAN_P);
	printf("\r\n2 c.(x-H*S):");	printfMatrix(&S_temp[0], KALMAN_P);
	
	//2d.S1_temp=K(x-H*A*S)	[P]
	for(i=0;i<KALMAN_P;i++)
		{
		S1_temp[i]=0;
		for(count1=0;count1<KALMAN_M;count1++)
			{
			S1_temp[i]+=((k_filter.K[i*KALMAN_M+count1]*S_temp[count1])/1000);
			}
		}
	printf("\r\n2 d:");	printfMatrix(&S1_temp[0], KALMAN_P);
	
	//S=A*S+K(x-H*A*S)	[P]
	//--因为s[n|n-1]==s[n|n],所以A*S直接为S
	matrix_sum(&(k_filter.S[0]),&(S1_temp[0]),&(k_filter.S[0]),1,KALMAN_P);
	printf("\r\n2-.S:");	printfMatrix(&k_filter.S[0], KALMAN_P);
}

/*-------------------
- 最小MSE矩阵
---------------------*/
void minimum_MSE(void)
{
	uint8_t i;
	//3a.PP_temp=K*H
	matrix_mul(&(k_filter.K[0]),&(k_filter.H[0]),&(k_filter.PP_temp[0]),KALMAN_P,KALMAN_P);
	printf("\r\n3 a:");	printfMatrix(&k_filter.PP_temp[0], KALMAN_P*KALMAN_P);
	
	//3b.I-K*H
	for(i=0;i<KALMAN_P;i++)
		{
		k_filter.PP_temp[i*KALMAN_P+i]=1000-k_filter.PP_temp[i*KALMAN_P+i];
		}
	printf("\r\n3 b:");	printfMatrix(&k_filter.PP_temp[0], KALMAN_P*KALMAN_P);
	
	//(I-K*H)*M_pre
	matrix_mul(&(k_filter.PP_temp[0]),&(k_filter.M_pre[0]),&(k_filter.M[0]),KALMAN_P,KALMAN_P);
	printf("\r\n3-.M:");	printfMatrix(&k_filter.M[0], KALMAN_P*KALMAN_P);
}

/*-------------------
- 最小预测MSE矩阵
---------------------*/
void minimum_prediction_MSE(void)
{
	uint8_t i,j,count,count1;
/*	//A*M	[p*p]
	matrix_mul(&(k_filter.A[0]),&(k_filter.M[0]),&(k_filter.PP_temp[0]),KALMAN_P,KALMAN_P);
	
	//A*M*A(T)	[p*p]
	for(i=0;i<KALMAN_P;i++)
		{
		for(j=0;j<KALMAN_P;j++)
			{
			count=i*KALMAN_P+j;
			k_filter.M_pre[count]=0;
			for(count1=0;count1<KALMAN_P;count1++)
				{
				k_filter.M_pre[count]+=((k_filter.PP_temp[i*KALMAN_P+count1]*k_filter.M[j*KALMAN_P+count1])/1000);
				}
			k_filter.M_pre[count]+=k_filter.Q[count];
			}
		}*/
	matrix_sum(&k_filter.M[0], &k_filter.Q[0],&k_filter.M_pre[0], KALMAN_P, KALMAN_P);
	printf("\r\n4-.M_pre:");	printfMatrix(&k_filter.M_pre[0], KALMAN_P*KALMAN_P);
}

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
	uint8_t i=0;	//a:当前计算到的元素 列的计数 a=0, 
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
			c_temp+=(a_temp*b_temp)/1000;
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
	uint8_t rows[MATRIX_ORDER]={0};
	uint8_t columns[MATRIX_ORDER]={0};
	MATRIX_DATA_TYPE d,p;	  //,a
//	MATRIX_DATA_TYPE * temp;

	for(k=0;k<MATRIX_AMOUNT;k++)
		{
		*matrix_C = *matrix_A;
		matrix_C += BIT32_ADD_OFFSET;
		matrix_A += BIT32_ADD_OFFSET;
		}
	matrix_C -= 36*BIT32_ADD_OFFSET;
	matrix_A -= 36*BIT32_ADD_OFFSET;

	for(k=0;k<MATRIX_ORDER;k++)
		{
		d=0;
		for(row=k;row<MATRIX_ORDER;row++)
			{
			for(column=k;column<MATRIX_ORDER;column++)
				{
				l=row*MATRIX_ORDER+column;
				p=abs(*(matrix_C+l*BIT32_ADD_OFFSET));
				if(p>d)
					{
					d=p;
					rows[k]=row;
					columns[k]=column;
//					printf("\r\n$k:%d $d:%d ",k,d);
					}
				}
			}
		//根据上面的判断交换行和列
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
//		temp=matrix_C+l*BIT32_ADD_OFFSET;
//		p=(*(matrix_C+l*BIT32_ADD_OFFSET));
//		p=(1000*1000)/p;
//		*(matrix_C+l*BIT32_ADD_OFFSET)=p;
		//a[kk]=1/a[kk]
		*(matrix_C+l*BIT32_ADD_OFFSET) = (1000*1000)/(*(matrix_C+l*BIT32_ADD_OFFSET));	//, 1=>1000

		for(column=0;column<MATRIX_ORDER;column++)
			{
			if(column!=k)
				{
				u=k*MATRIX_ORDER+column;
				//a[kj]=a[kj]a[kk]
				*(matrix_C+u*BIT32_ADD_OFFSET)=(((*(matrix_C+u*BIT32_ADD_OFFSET))*(*(matrix_C+l*BIT32_ADD_OFFSET)))/1000);	
				}
			}
		for(row=0;row<MATRIX_ORDER;row++)
			{
			if(row!=k)
				{
				for(column=0;column<MATRIX_ORDER;column++)
					{
					if(column!=k)
						{
						u=row*MATRIX_ORDER+column;
						//a[ij]=a[ij]-a[ik]a[kj]
//						a=(*(matrix_C+(row*MATRIX_ORDER+k)*BIT32_ADD_OFFSET));
//						p=(*(matrix_C+(k*MATRIX_ORDER+column)*BIT32_ADD_OFFSET));
						(*(matrix_C+u*BIT32_ADD_OFFSET)) = (*(matrix_C+u*BIT32_ADD_OFFSET)) -\
						(((*(matrix_C+(row*MATRIX_ORDER+k)*BIT32_ADD_OFFSET))*(*(matrix_C+(k*MATRIX_ORDER+column)*BIT32_ADD_OFFSET)))/1000);
//						*(matrix_C+u*BIT32_ADD_OFFSET) = (*(matrix_C+u*BIT32_ADD_OFFSET))-((a*p)/1000);
						}
					}
				}
			}
		for(row=0;row<MATRIX_ORDER;row++)
			{
			if(row!=k)
				{
				u=row*MATRIX_ORDER+k;
				//a[ik]=-a[ik]a[kk]
				*(matrix_C+u*BIT32_ADD_OFFSET)=(-(*(matrix_C+u*BIT32_ADD_OFFSET))*(*(matrix_C+l*BIT32_ADD_OFFSET)))/1000;
//				*(matrix_C+u*BIT32_ADD_OFFSET)=(*(matrix_C+u*BIT32_ADD_OFFSET))/1000;
				}
			}
		}
	
	for(k=MATRIX_ORDER-1;k>=0;k--)	//恢复矩阵
		{
		if(columns[k]!=k)
			{
			for(column=0;column<MATRIX_ORDER;column++)
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
			for(row=0;row<MATRIX_ORDER;row++)
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

/*-------------------
- 数值转换 32->8
---------------------*/
void int32ToChar(int32_t *data32, uint8_t *data8, int16_t data32size, int16_t data8Offset)
{
	int16_t i,temp16;
	int16_t data8Addr=data8Offset;
	for(i=0;i<data32size;i++)
		{
		temp16 = (int16_t)*(data32+i);	//丢弃高16位
//		printf("temp16: %d",temp16);
		*(data8+data8Addr)=temp16&0xFF;
		data8Addr++;
		*(data8+data8Addr)=(temp16>>8)&0xFF;
		data8Addr++;
		}
}

/*-------------------
- 浮点卡尔曼滤波器
输出:加速度(g-float), 角速度(dps-float)
---------------------*/
void kalmanFilterFloat(void)
{
	int16_t i;
	
	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.PP_temp[i]=kFFloat.M_pre[i]*kFFloat.H[i];
		
		kFFloat.PM_temp[i]=kFFloat.H[i]*kFFloat.M_pre[i]*kFFloat.H[i];
		kFFloat.PM_temp[i]+=kFFloat.C[i];

		kFFloat.K[i]=kFFloat.PP_temp[i] / kFFloat.PM_temp[i];	//K
		}

	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.PP_temp[i]=kFFloat.x[i]-(kFFloat.H[i]*kFFloat.S[i]);
		kFFloat.PM_temp[i]=kFFloat.K[i]*kFFloat.PP_temp[i];

		kFFloat.S[i]=kFFloat.S[i]+kFFloat.PM_temp[i];	//S[n|n]
		}

	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.PP_temp[i]=1.0-(kFFloat.K[i]*kFFloat.H[i]);
		kFFloat.M[i]=kFFloat.PP_temp[i]*kFFloat.M_pre[i];	//M
		}
	
	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.M_pre[i]=kFFloat.M[i]+kFFloat.Q[i];		//M_pre
		}	

	kFFloat.PP_temp[0]=kFFloat.S[0];
	kFFloat.PP_temp[1]=kFFloat.S[1];
	kFFloat.PP_temp[2]=kFFloat.S[2];
//	kFFloat.PP_temp[3]=kFFloat.S[3]-kFFloat.S_pre[3];
//	kFFloat.PP_temp[4]=kFFloat.S[4]-kFFloat.S_pre[4];
//	kFFloat.PP_temp[5]=kFFloat.S[5]-kFFloat.S_pre[5];	//单位时间内 角度增加
	kFFloat.PP_temp[3]=kFFloat.S[3]*10.0*ANGtoRAD;
	kFFloat.PP_temp[4]=kFFloat.S[4]*10.0*ANGtoRAD;
	kFFloat.PP_temp[5]=kFFloat.S[5]*10.0*ANGtoRAD;
}

/*-------------------
- 浮点卡尔曼滤波器参数赋值
---------------------*/
void kalmanFilterFloatInit(void)
{
	int i;
	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.S[i]=0.0;
		}
	for(i=0;i<KALMAN_P;i++)
		{
		kFFloat.A[i]=0.0;
		kFFloat.Q[i]=0.0;
		kFFloat.H[i]=0.0;
		kFFloat.C[i]=0.0;
		}
	
	//初始化A
//	kFFloat.A[0]=1.0; kFFloat.A[1]=1.0; kFFloat.A[2]=1.0;	//1.000 = 1000
//	kFFloat.A[3]=1.0;kFFloat.A[4]=1.0;kFFloat.A[5]=1.0;

	//初始化Q
	//--假设Q为0
	kFFloat.Q[0]=KALMAN_Q_FLOAT; kFFloat.Q[1]=KALMAN_Q_FLOAT; kFFloat.Q[2]=KALMAN_Q_FLOAT;	//1.000 = 1000
	kFFloat.Q[3]=KALMAN_Q_FLOAT;kFFloat.Q[4]=KALMAN_Q_FLOAT;kFFloat.Q[5]=KALMAN_Q_FLOAT;
	
	//初始化H
	kFFloat.H[0]=16967.0; kFFloat.H[1]=16967.0; kFFloat.H[2]=16967.0;
//	kFFloat.H[3]=11428;kFFloat.H[4]=11428;kFFloat.H[5]=11428;
	kFFloat.H[3]=KALMAN_G_H;kFFloat.H[4]=KALMAN_G_H;kFFloat.H[5]=KALMAN_G_H;

	
	//初始化C
	//	k_filter.C[0]=7284;k_filter.C[7]=7284;k_filter.C[14]=7284;
	//	k_filter.C[21]=4784;k_filter.C[28]=4784;k_filter.C[35]=4784;
	kFFloat.C[0]=KALMAN_A_C_FLOAT;kFFloat.C[1]=KALMAN_A_C_FLOAT;kFFloat.C[2]=KALMAN_A_C_FLOAT;
	kFFloat.C[3]=KALMAN_G_C_FLOAT;kFFloat.C[4]=KALMAN_G_C_FLOAT;kFFloat.C[5]=KALMAN_G_C_FLOAT;

	
	//初始化M
	kFFloat.M[0]=KALMAN_A_C_FLOAT;kFFloat.M[1]=KALMAN_A_C_FLOAT;kFFloat.M[2]=KALMAN_A_C_FLOAT;
	kFFloat.M[3]=KALMAN_G_C_FLOAT;kFFloat.M[4]=KALMAN_G_C_FLOAT;kFFloat.M[5]=KALMAN_G_C_FLOAT;

}

/*-------------------
- 数值转换 float->8
---------------------*/
void floatToChar(float *data32, uint8_t *data8, int16_t data32size, int16_t data8Offset)
{
	int16_t i,temp16;
	int16_t data8Addr=data8Offset;
	uint8_t tempChar;

	for(i=0;i<data32size;i++)
		{
		temp16 = (int16_t)(*(data32+i)*1000);	//丢弃高16位
								
		tempChar=temp16&0xFF;
		*(data8+data8Addr)=tempChar;
		data8Addr++;			 
		tempChar=(temp16>>8)&0xFF;
		*(data8+data8Addr)=tempChar;
		data8Addr++;
		}


}


/*-------------------
- quaternion 四元数
---------------------*/
void quaternion(void)
{
//	int i;
//	float temp1,temp2;
	float normalizeDenominator;
	float tNew,xNew,yNew,zNew;
//	float tPre,xPre,yPre,zPre;
	float sinP,cosP,sinR,cosR,sinY,cosY;
	
	normalizeDenominator=sqrt(quat.t*quat.t+quat.x*quat.x+quat.y*quat.y+quat.z*quat.z);
	quat.t=quat.t/normalizeDenominator;
	quat.x=quat.x/normalizeDenominator;
	quat.y=quat.y/normalizeDenominator;
	quat.z=quat.z/normalizeDenominator;
	cosP=cos(kFFloat.PP_temp[3]*0.5);	//x
	sinP=sin(kFFloat.PP_temp[3]*0.5);
	cosR=cos(kFFloat.PP_temp[4]*0.5);	//y
	sinR=sin(kFFloat.PP_temp[4]*0.5);
	cosY=cos(kFFloat.PP_temp[5]*0.5);	//z
	sinY=sin(kFFloat.PP_temp[5]*0.5);

	tNew = cosR*cosP*cosY + sinR*sinP*sinY;
	xNew = cosR*sinP*cosY + sinR*cosP*sinY;
	yNew = cosR*cosP*sinY - sinR*sinP*cosY;
	zNew = sinR*cosP*cosY - cosR*sinP*sinY;
	quat.t = quat.t*tNew -quat.x*xNew -quat.y*yNew -quat.z*zNew;
	quat.x = quat.t*xNew +quat.x*tNew +quat.y*zNew -quat.z*yNew;
	quat.y = quat.t*yNew -quat.x*zNew +quat.y*tNew +quat.z*xNew;
	quat.z = quat.t*zNew +quat.x*yNew -quat.y*xNew +quat.z*tNew;


	//pitch
	quat.pitch=asin(2.0*(quat.t*quat.x-quat.y*quat.z));

	//roll
	quat.roll=atan2(2.0*(quat.t*quat.z+quat.x*quat.y), 1.0-2.0*(quat.z*quat.z+quat.x*quat.x));
	
	//yaw
	quat.yaw=atan2(2*(quat.t*quat.y+quat.z*quat.x), 1.0-2*(quat.x*quat.x+quat.y*quat.y));

}


/*-------------------
- 四元数 参数赋值
---------------------*/
void quaternionInit(void)
{
	quat.t=1.0;
	quat.x=0.0;
	quat.y=0.0;
	quat.z=0.0;
}


