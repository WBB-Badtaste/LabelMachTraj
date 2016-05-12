#define S_FUNCTION_NAME  cubic
#define S_FUNCTION_LEVEL 2
#include "malloc.h"//����ʹ�ñ������������С

void TDMA(double* X, const int n, double* A, double* B, double* C, double* D)
{
	int i;
	double tmp;

	//�����Ǿ���
	C[0] = C[0] / B[0];
	D[0] = D[0] / B[0];

	for(i = 1; i<n; i++)
	{
		tmp  = (B[i] - A[i] * C[i - 1]);
		C[i] = C[i] / tmp;
		D[i] = (D[i] - A[i] * D[i - 1]) / tmp;
	}

	//ֱ�����X�����һ��ֵ
	X[n - 1] = D[n - 1];

	//��������� ���X
	for(i = n - 2; i >= 0; i--)
	{
		X[i] = D[i] - C[i] * X[i + 1];
	}
}

//��Ȼ�߽�������������ߺ���
void cubic_getval(const int n, const double* map, const double x, double *pars = nullptr)
{
    //����ϵ��
	if (!pars)
		pars = (double*)malloc(sizeof(double) * (n-1) * 4);
    
	double* h = (double*)malloc(sizeof(double) * (n - 1));  //x��?
	/* M�����ϵ��
	*[B0, C0, ...
	*[A1, B1, C1, ...
	*[0,  A2, B2, C2, ...
	*[0, ...             An-1, Bn-1]
	*/
	double* A = (double*)malloc(sizeof(double) * (n - 2)); 
	double* B = (double*)malloc(sizeof(double) * (n - 2)); 
	double* C = (double*)malloc(sizeof(double) * (n - 2)); 
	double* D = (double*)malloc(sizeof(double) * (n - 2)); //�Ⱥ��ұߵĳ�������
	double* E = (double*)malloc(sizeof(double) * (n - 2)); //M����
	double* M = (double*)malloc(sizeof(double) * (n));	   //�����˵��M����

    int i; 
    
    //����x�Ĳ���
    for ( i = 0; i < n -1; i++)
    {
        h[i] = map[i + 1] - map[i];
    }
    
    //ָ��ϵ��
    for( i = 0; i< n - 3; i++)
	{
        A[i] = h[i];	//����A[0]
        B[i] = 2 * (h[i] + h[i+1]);
        C[i] = h[i+1];	//����C(n-1)
    }
    
    //ָ������D
    for (i = 0; i<n - 3; i++)
    {
        D[i] = 6 * ((map[n + i + 2] - map[n + i + 1]) / h[i + 1] - (map[n + i + 1] - map[n + i]) / h[i]);
    }
    
    //������ԽǾ��󣬽����ֵ��E
    TDMA(E, n-3, A, B, C, D);
    
    M[0]   = 0; //��Ȼ�߽���׶�MΪ0
    M[n-1] = 0; //��Ȼ�߽��ĩ��MΪ0
    for(i=1; i<n-1; i++)
    {
        M[i] = E[i-1]; //������Mֵ
    }
    
    //�����������������ߵ�ϵ��
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


