#include "matrix.h"

void* CreateMatrix(int rows,int colums)
{
    Matrix_t* matrix=(Matrix_t*)malloc(sizeof(Matrix_t)+rows*colums*sizeof(float));
    matrix->rows=rows;
    matrix->columns=colums;
    memset(matrix->data,0,rows*colums*sizeof(float));
    return matrix;
}

int MatrixProduct(Matrix_t* dst, Matrix_t* m1, Matrix_t* m2)
{
    if (m1->columns != m2->rows || dst->rows != m1->rows || dst->columns != m2->columns)
        return -1;

    for (int i = 0; i < dst->rows; ++i)
    {
        for (int j = 0; j < dst->columns; ++j)
        {
            float sum = 0.0f;
            for (int k = 0; k < m1->columns; ++k)
            {
                sum += GetElement(m1, i, k) * GetElement(m2, k, j);
            }
            SetElement(dst, i, j, sum);
        }
    }
    return 0;
}

int MatrixAdd(Matrix_t* dst, Matrix_t* m1, Matrix_t* m2)
{
    if (m1->rows != m2->rows || m1->columns != m2->columns ||
        dst->rows != m1->rows || dst->columns != m1->columns)
        return -1;

    int totalElements = m1->rows * m1->columns;
    for (int i = 0; i < totalElements; ++i)
    {
        dst->data[i] = m1->data[i] + m2->data[i];
    }
    return 0;
}

int MatrixTranspose(Matrix_t* dst, Matrix_t* m)
{
    if (dst->rows != m->columns || dst->columns != m->rows)
        return -1;

    for (int i = 0; i < m->rows; ++i)
    {
        for (int j = 0; j < m->columns; ++j)
        {
            SetElement(dst, j, i, GetElement(m, i, j));
        }
    }
    return 0;
}

int MatrixInverse(Matrix_t* dst, Matrix_t* m)
{
    if (m->rows != m->columns || dst->rows != m->rows || dst->columns != m->columns)
        return -1;

    int n = m->rows;

    if (n == 2)
    {
        float a = GetElement(m, 0, 0);
        float b = GetElement(m, 0, 1);
        float c = GetElement(m, 1, 0);
        float d = GetElement(m, 1, 1);

        float det = a * d - b * c;
        if (det == 0.0f) return -2;  // 不可逆

        SetElement(dst, 0, 0,  d / det);
        SetElement(dst, 0, 1, -b / det);
        SetElement(dst, 1, 0, -c / det);
        SetElement(dst, 1, 1,  a / det);
        return 0;
    }
    else if (n == 3)
    {
        float a = GetElement(m, 0, 0), b = GetElement(m, 0, 1), c = GetElement(m, 0, 2);
        float d = GetElement(m, 1, 0), e = GetElement(m, 1, 1), f = GetElement(m, 1, 2);
        float g = GetElement(m, 2, 0), h = GetElement(m, 2, 1), i = GetElement(m, 2, 2);

        float det = a*(e*i - f*h) - b*(d*i - f*g) + c*(d*h - e*g);
        if (det == 0.0f) return -2;  // 不可逆

        SetElement(dst, 0, 0,  (e*i - f*h) / det);
        SetElement(dst, 0, 1, -(b*i - c*h) / det);
        SetElement(dst, 0, 2,  (b*f - c*e) / det);
        SetElement(dst, 1, 0, -(d*i - f*g) / det);
        SetElement(dst, 1, 1,  (a*i - c*g) / det);
        SetElement(dst, 1, 2, -(a*f - c*d) / det);
        SetElement(dst, 2, 0,  (d*h - e*g) / det);
        SetElement(dst, 2, 1, -(a*h - b*g) / det);
        SetElement(dst, 2, 2,  (a*e - b*d) / det);
        return 0;
    }

    return -3; // 不支持更高阶
}

void DeleteMatrix(Matrix_t* m)
{
    free(m);
}

void PrintMatrix(Matrix_t* m)
{
    for (int i = 0; i < m->rows; ++i)
    {
        for (int j = 0; j < m->columns; ++j)
        {
            //printf("%.2f ", GetElement(m, i, j));
        }
        //printf("\n");
    }
}
