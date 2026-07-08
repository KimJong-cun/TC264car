/*
 * matrix.c
 *
 *  Matrix helpers for EKF
 *  Copyright: KimJong
 */

#include "matrix.h"

void Matrix_Init(matrix_t *martix, int rows, int cols)
{
    ASSERT(rows > 0 && cols > 0);
    martix->rows = rows;
    martix->cols = cols;
    memset(martix->data, 0, MAX_SIZE * MAX_SIZE * sizeof(matrix_type));
}

void Matrix_Identity(matrix_t *matrix, int size)
{
    int i;

    ASSERT(size > 0);
    matrix->rows = size;
    matrix->cols = size;
    memset(matrix->data, 0, sizeof(matrix->data));
    for (i = 0; i < size; i++)
    {
        matrix->data[i][i] = 1.0f;
    }
}

void Matrix_From_Array(matrix_t *mat, const matrix_type *array, const int rows, const int cols)
{
    int i;
    int j;

    ASSERT(NULL != array);
    Matrix_Init(mat, rows, cols);
    for (i = 0; i < rows; ++i)
    {
        for (j = 0; j < cols; ++j)
        {
            mat->data[i][j] = array[i * cols + j];
        }
    }
}

matrix_t Matrix_Transpose(const matrix_t *src)
{
    int i;
    int j;
    matrix_t dest;

    Matrix_Init(&dest, src->cols, src->rows);
    for (i = 0; i < src->rows; i++)
    {
        for (j = 0; j < src->cols; j++)
        {
            dest.data[j][i] = src->data[i][j];
        }
    }
    return dest;
}

matrix_t multiply_matrices(const matrix_t *A, const matrix_t *B)
{
    int i;
    int j;
    int k;
    matrix_t dest;

    ASSERT(A->cols == B->rows);
    Matrix_Init(&dest, A->rows, B->cols);

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < B->cols; j++)
        {
            for (k = 0; k < A->cols; k++)
            {
                dest.data[i][j] += A->data[i][k] * B->data[k][j];
            }
        }
    }

    return dest;
}

matrix_t add_matrices(const matrix_t *A, const matrix_t *B)
{
    int i;
    int j;
    matrix_t result;

    ASSERT(A->rows == B->rows && A->cols == B->cols);
    Matrix_Init(&result, A->rows, A->cols);

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] + B->data[i][j];
        }
    }

    return result;
}

matrix_t subtract_matrices(const matrix_t *A, const matrix_t *B)
{
    int i;
    int j;
    matrix_t result;

    ASSERT(A->rows == B->rows && A->cols == B->cols);
    Matrix_Init(&result, A->rows, A->cols);

    for (i = 0; i < A->rows; i++)
    {
        for (j = 0; j < A->cols; j++)
        {
            result.data[i][j] = A->data[i][j] - B->data[i][j];
        }
    }

    return result;
}

int inverse_matrix(matrix_t *A, matrix_t *invA)
{
    const matrix_type THRESHOLD = 1e-6f;
    matrix_type augmented[MAX_SIZE][2 * MAX_SIZE];
    int n;
    int i;
    int j;
    int k;

    ASSERT(A->rows == A->cols);
    Matrix_Init(invA, A->rows, A->cols);

    n = A->rows;
    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            augmented[i][j] = A->data[i][j];
            augmented[i][j + n] = (matrix_type)((i == j) ? 1 : 0);
        }
    }

    for (i = 0; i < n; i++)
    {
        int max_row = i;

        for (j = i + 1; j < n; j++)
        {
            if (fabs(augmented[j][i]) > fabs(augmented[max_row][i]))
            {
                max_row = j;
            }
        }

        if (fabs(augmented[max_row][i]) < THRESHOLD)
        {
            return 1;
        }

        if (max_row != i)
        {
            for (j = 0; j < 2 * n; j++)
            {
                matrix_type temp = augmented[i][j];
                augmented[i][j] = augmented[max_row][j];
                augmented[max_row][j] = temp;
            }
        }

        {
            matrix_type pivot = augmented[i][i];
            for (j = 0; j < 2 * n; j++)
            {
                augmented[i][j] /= pivot;
            }
        }

        for (j = 0; j < n; j++)
        {
            if (j != i)
            {
                matrix_type factor = augmented[j][i];
                for (k = 0; k < 2 * n; k++)
                {
                    augmented[j][k] -= factor * augmented[i][k];
                }
            }
        }
    }

    for (i = 0; i < n; i++)
    {
        for (j = 0; j < n; j++)
        {
            invA->data[i][j] = augmented[i][j + n];
        }
    }

    return 0;
}

static inline float invSqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int *)&x;

    i = 0x5f375a86 - (i >> 1);
    x = *(float *)&i;
    x = x * (1.5f - xhalf * x * x);

    return x;
}

void normalize_vector(matrix_t *v)
{
    int i;
    matrix_type norm = 0;

    ASSERT(1 == v->cols || 1 == v->rows);

    if (1 == v->rows)
    {
        for (i = 0; i < v->cols; ++i)
        {
            norm += (v->data[0][i] * v->data[0][i]);
        }
    }
    if (1 == v->cols)
    {
        for (i = 0; i < v->rows; ++i)
        {
            norm += (v->data[i][0] * v->data[i][0]);
        }
    }

    norm = invSqrt((float)norm);
    if (1 == v->rows)
    {
        for (i = 0; i < v->cols; ++i)
        {
            v->data[0][i] *= norm;
        }
    }
    if (1 == v->cols)
    {
        for (i = 0; i < v->rows; ++i)
        {
            v->data[i][0] *= norm;
        }
    }
}

void print_matrix(const matrix_t *matrix)
{
    int i;
    int j;

    for (i = 0; i < matrix->rows; i++)
    {
        for (j = 0; j < matrix->cols; j++)
        {
            my_printf(WIRELESS_UART_INDEX, "%2f ", matrix->data[i][j]);
        }
        my_printf(WIRELESS_UART_INDEX, "\n");
    }
    my_printf(WIRELESS_UART_INDEX, "\n");
}
