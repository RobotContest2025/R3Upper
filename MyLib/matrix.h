#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    int rows;       //行数
    int columns;    //列数
    float data[];
}Matrix_t;


static inline float GetElement(Matrix_t* m, int row, int col)
{
    return m->data[row * m->columns + col];
}

static inline void SetElement(Matrix_t* m, int row, int col, float value)
{
    m->data[row * m->columns + col] = value;
}

void* CreateMatrix(int rows,int colums);
int MatrixProduct(Matrix_t* dst, Matrix_t* m1, Matrix_t* m2);
int MatrixAdd(Matrix_t* dst, Matrix_t* m1, Matrix_t* m2);
int MatrixTranspose(Matrix_t* dst, Matrix_t* m);
int MatrixInverse(Matrix_t* dst, Matrix_t* m);
void DeleteMatrix(Matrix_t* m);
void PrintMatrix(Matrix_t* m);

#define FillMatrix2d(matrix,a11,a12,a21,a22)\
SetElement(matrix, 0, 0, a11);\
SetElement(matrix, 0, 1, a12);\
SetElement(matrix, 1, 0, a21);\
SetElement(matrix, 1, 1, a22)

#define FillMatrix3d(matrix,a11,a12,a13,a21,a22,a23,a31,a32,a33)\
SetElement(matrix, 0, 0, a11);\
SetElement(matrix, 0, 1, a12);\
SetElement(matrix, 0, 2, a13);\
SetElement(matrix, 1, 0, a21);\
SetElement(matrix, 1, 1, a22);\
SetElement(matrix, 1, 2, a23);\
SetElement(matrix, 2, 0, a31);\
SetElement(matrix, 2, 1, a32);\
SetElement(matrix, 2, 2, a33)

#endif
