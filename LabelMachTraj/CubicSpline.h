#define S_FUNCTION_NAME  cubic
#define S_FUNCTION_LEVEL 2
#include "malloc.h"//方便使用变量定义数组大小

void TDMA(double* X, const int n, double* A, double* B, double* C, double* D)
{
	int i;
	double tmp;

	//上三角矩阵
	C[0] = C[0] / B[0];
	D[0] = D[0] / B[0];

	for(i = 1; i<n; i++)
	{
		tmp  = (B[i] - A[i] * C[i - 1]);
		C[i] = C[i] / tmp;
		D[i] = (D[i] - A[i] * D[i - 1]) / tmp;
	}

	//直接求出X的最后一个值
	X[n - 1] = D[n - 1];

	//逆向迭代， 求出X
	for(i = n - 2; i >= 0; i--)
	{
		X[i] = D[i] - C[i] * X[i + 1];
	}
}

//自然边界的三次样条曲线函数
void cubic_getval(const int n, const double* map, const double x, double *pars = nullptr)
{
    //曲线系数
	if (!pars)
		pars = (double*)malloc(sizeof(double) * (n-1) * 4);
    
	double* h = (double*)malloc(sizeof(double) * (n - 1));  //x的?
	/* M矩阵的系数
	*[B0, C0, ...
	*[A1, B1, C1, ...
	*[0,  A2, B2, C2, ...
	*[0, ...             An-1, Bn-1]
	*/
	double* A = (double*)malloc(sizeof(double) * (n - 2)); 
	double* B = (double*)malloc(sizeof(double) * (n - 2)); 
	double* C = (double*)malloc(sizeof(double) * (n - 2)); 
	double* D = (double*)malloc(sizeof(double) * (n - 2)); //等号右边的常数矩阵
	double* E = (double*)malloc(sizeof(double) * (n - 2)); //M矩阵
	double* M = (double*)malloc(sizeof(double) * (n));	   //包含端点的M矩阵

    int i; 
    
    //计算x的步长
    for ( i = 0; i < n -1; i++)
    {
        h[i] = map[i + 1] - map[i];
    }
    
    //指定系数
    for( i = 0; i< n - 3; i++)
	{
        A[i] = h[i];	//忽略A[0]
        B[i] = 2 * (h[i] + h[i+1]);
        C[i] = h[i+1];	//忽略C(n-1)
    }
    
    //指定常数D
    for (i = 0; i<n - 3; i++)
    {
        D[i] = 6 * ((map[n + i + 2] - map[n + i + 1]) / h[i + 1] - (map[n + i + 1] - map[n + i]) / h[i]);
    }
    
    //求解三对角矩阵，结果赋值给E
    TDMA(E, n-3, A, B, C, D);
    
    M[0]   = 0; //自然边界的首端M为0
    M[n-1] = 0; //自然边界的末端M为0
    for(i=1; i<n-1; i++)
    {
        M[i] = E[i-1]; //其它的M值
    }
    
    //计算算三次样条曲线的系数
    for( i = 0; i < n-1; i++)
    {
        pars[(n - 1) * 0 + i] = map[n + i];
        pars[(n - 1) * 1 + i] = (map[n + i + 1] - map[n + i]) / h[i] - (2 * h[i] * M[i] + h[i] * M[i + 1]) / 6;
        pars[(n - 1) * 2 + i] = M[i] / 2;
        pars[(n - 1) * 3 + i] = (M[i + 1] - M[i]) / (6 * h[i]);
    }
    
    free(h);
    free(A);
    free(B);
    free(C);
    free(D);
    free(E);
    free(M);
}


