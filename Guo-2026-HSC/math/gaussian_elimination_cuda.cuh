#pragma once
#include "math/math_cuda.cuh"

template <typename Real>
__host__ __device__ __forceinline__
void gaussian_elimination_pivoting(Real* A, Real* b, Real* x, unsigned int n)
{
    // Augmented matrix: [A | b] stored in row-major 1D array
    Real* augmented = (Real*)malloc(sizeof(Real) * n * (n + 1));
    for (unsigned int i = 0; i < n; ++i)
    {
        for (unsigned int j = 0; j < n; ++j)
        {
            augmented[i * (n + 1) + j] = A[i * n + j]; // Copy A
        }
        augmented[i * (n + 1) + n] = b[i]; // Copy b
    }

    //printf("before GE ------------------------------------------------\n");
    //for (unsigned int i = 0; i < n; ++i)
    //{
    //    for (unsigned int j = 0; j < n + 1; ++j)
    //    {
    //        printf("%f ", augmented[i * (n + 1) + j]);
    //    }
    //    printf("\n");
    //}
    //printf("before GE ------------------------------------------------\n");

    // Gaussian Elimination with Partial Pivoting
    for (unsigned int i = 0; i < n; ++i) {
        // Partial Pivoting: Find the row with the maximum element in the current column
        unsigned int maxRow = i;
        Real maxVal = abs<Real>(augmented[i * (n + 1) + i]);
        for (unsigned int k = i + 1; k < n; ++k)
        {
            if (abs<Real>(augmented[k * (n + 1) + i]) > maxVal)
            {
                maxVal = abs<Real>(augmented[k * (n + 1) + i]);
                maxRow = k;
            }
        }

        // Swap the current row with the maxRow
        if (maxRow != i)
        {
            for (unsigned int j = 0; j <= n; ++j)
            {
                swap<Real>(augmented[i * (n + 1) + j], augmented[maxRow * (n + 1) + j]);
            }
        }

        // Check for singular matrix
        if (abs<Real>(augmented[i * (n + 1) + i]) < EPSILON)
        {
            printf("ERROR::gaussian_elimination_pivoting: Matrix is singular or nearly singular!\n");
            continue;
        }

        // Eliminate the current column elements below the pivot
        for (unsigned int k = i + 1; k < n; ++k)
        {
            Real factor = augmented[k * (n + 1) + i] / augmented[i * (n + 1) + i];
            for (unsigned int j = i; j <= n; ++j) {
                augmented[k * (n + 1) + j] -= factor * augmented[i * (n + 1) + j];
            }
        }
    }

    //printf("after GE ------------------------------------------------\n");
    //for (unsigned int i = 0; i < n; ++i)
    //{
    //    for (unsigned int j = 0; j < n + 1; ++j)
    //    {
    //        printf("%f ", augmented[i * (n + 1) + j]);
    //    }
    //    printf("\n");
    //}
    //printf("after GE ------------------------------------------------\n");

    // Back substitution to solve for x
    for (int i = n - 1; i >= 0; --i)
    {
        x[i] = augmented[i * (n + 1) + n] / augmented[i * (n + 1) + i];
        for (int k = i - 1; k >= 0; --k)
        {
            augmented[k * (n + 1) + n] -= augmented[k * (n + 1) + i] * x[i];
        }
    }

    //for (unsigned int i = 0; i < n; ++i)
    //{
    //    for (unsigned int j = 0; j < n + 1; ++j)
    //    {
    //        printf("%f ", augmented[(n + 1) * i + j]);
    //    }
    //    printf("\n");
    //}

    free(augmented);
}

/*
Current Gaussian elimination function incorporates partial pivoting and checks for singular matrices by
skipping elimination when the pivot is too small. However, skipping elimination in singular cases without
a fallback strategy leads to incomplete row echelon form, which might not be ideal for downstream operations.
Generalization Strategy for Singular Matrices
Track Rank Deficiency: Identify linearly dependent rows by checking when a pivot is nearly zero.
Row Exchange Handling: Instead of skipping elimination, continue reducing independent rows while keeping dependent rows for later analysis.
Null Space Consideration: If needed, extract the null space basis when the rank is lower than the number of variables.
*/
template<typename Real>
__global__ void gaussian_elimination_pivoting_kernel(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    gaussian_elimination_pivoting<Real>(A, b, x, mat_size);
}

template <typename Real>
void gaussian_elimination_pivoting_cuda(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    gaussian_elimination_pivoting_kernel<Real> << <1, 1 >> > (A, b, x, mat_size);
}

template <typename Real>
__host__ __device__ __forceinline__
void gaussian_elimination_lhs_pivoting(Real* A, unsigned int n)
{
    Real* augmented = A;
    // Gaussian Elimination with Partial Pivoting
    for (unsigned int i = 0; i < n; ++i) {
        // Partial Pivoting: Find the row with the maximum element in the current column
        unsigned int maxRow = i;
        Real maxVal = abs<Real>(augmented[i * n + i]);
        for (unsigned int k = i + 1; k < n; ++k)
        {
            if (abs<Real>(augmented[k * n + i]) > maxVal)
            {
                maxVal = abs<Real>(augmented[k * n + i]);
                maxRow = k;
            }
        }

        // Swap the current row with the maxRow
        if (maxRow != i)
        {
            for (unsigned int j = 0; j < n; ++j)
            {
                swap<Real>(augmented[i * n + j], augmented[maxRow * n + j]);
            }
        }

        // Check for singular matrix
        if (abs<Real>(augmented[i * n + i]) < EPSILON)
        {
            printf("ERROR::gaussian_elimination_lhs_pivoting: Matrix is singular or nearly singular!\n");
        }

        // Eliminate the current column elements below the pivot
        for (unsigned int k = i + 1; k < n; ++k)
        {
            Real factor = augmented[k * n + i] / augmented[i * n + i];
            for (unsigned int j = i; j < n; ++j) {
                augmented[k * n + j] -= factor * augmented[i * n + j];
            }
        }
    }
}

template <typename Real>
__host__ __device__ __forceinline__
void gaussian_elimination_lhs_pivoting(Real* A, unsigned int rows, unsigned int cols)
{
    // Gaussian Elimination with Partial Pivoting
    for (unsigned int i = 0; i < rows; ++i) {
        // Partial Pivoting: Find the row with the maximum element in the current column
        unsigned int maxRow = i;
        Real maxVal = abs<Real>(A[i * cols + i]);
        for (unsigned int k = i + 1; k < rows; ++k)
        {
            //printf("k = %d, i = %d\n", k, i);
            if (abs<Real>(A[k * cols + i]) > maxVal)
            {
                maxVal = abs<Real>(A[k * cols + i]);
                maxRow = k;
            }
        }

        // Swap the current row with the maxRow
        if (maxRow != i && maxRow < rows)
        {
            //printf("i = %d, maxRow = %d\n", i, maxRow);
            for (unsigned int j = 0; j < cols; ++j)
            {
                swap<Real>(A[i * cols + j], A[maxRow * cols + j]);
            }
        }

        // Check for singular matrix
        if (abs<Real>(A[i * cols + i]) < EPSILON)
        {
            //Matrix is singular or rank-deficient, continue to next pivot column
            continue;
        }

        // Eliminate the current column elements below the pivot
        for (unsigned int k = i + 1; k < rows; ++k)
        {
            Real factor = A[k * cols + i] / A[i * cols + i];
            for (unsigned int j = i; j < cols; ++j)
            {
                A[k * cols + j] -= factor * A[i * cols + j];
            }
        }
    }
}

template <typename Real>
__host__ __device__ __forceinline__
void gaussian_elimination_lhs_pivoting_inflate(Real* A, Real* A_inflate, unsigned int rows, unsigned int cols, unsigned int* row_ids)
{
    gaussian_elimination_lhs_pivoting<Real>(A, rows, cols);
    memset(A_inflate, 0., sizeof(Real) * cols * cols);

    for (unsigned int i = 0; i < cols; ++i)
    {
        A_inflate[cols * i + i] = 1.;
    }

    for (unsigned int i = 0; i < rows; ++i)
    {
        for (unsigned int j = 0; j < cols; ++j)
        {
            if (abs<Real>(A[cols * i + j]) > EPSILON)
            {
                row_ids[i] = j;
                for (unsigned int k = j; k < cols; ++k)
                {
                    A_inflate[cols * j + k] = A[cols * i + k];
                }
                break;
            }
        }
    }

    //for (unsigned int i = 0; i < cols; ++i)
    //{
    //    for (unsigned int j = 0; j < cols; ++j)
    //    {
    //        printf("%.0f ", A_inflate[cols * i + j]);
    //    }
    //    printf("\n");
    //}
}

template <typename Real>
__global__ void gaussian_elimination_lhs_pivoting_inflate_kernel(Real* A, Real* A_inflate, unsigned int rows, unsigned int cols, unsigned int* row_ids)
{
    gaussian_elimination_lhs_pivoting_inflate<Real>(A, A_inflate, rows, cols, row_ids);
}

template <typename Real>
void gaussian_elimination_lhs_pivoting_inflate_cuda(Real* A, Real* A_inflate, unsigned int rows, unsigned int cols, unsigned int* row_ids)
{
    gaussian_elimination_lhs_pivoting_inflate_kernel<Real> << <1, 1 >> > (A, A_inflate, rows, cols, row_ids);
}

template <typename Real>
__host__ __device__ __forceinline__
void upper_triangular_inverse(Real* U, unsigned int n, Real* inv_U)
{
    // Initialize inv_U as an identity matrix
    for (unsigned int i = 0; i < n; ++i) for (unsigned int j = 0; j < n; ++j)
    {
        inv_U[i * n + j] = (i == j) ? 1.0 : 0.0;
    }

    // Perform backward substitution
    for (int i = n - 1; i >= 0; --i)
    {
        if (U[i * n + i] == 0.0)
        {
            printf("Error: Matrix is singular and cannot be inverted.\n");
            //exit(1);
        }
        inv_U[i * n + i] /= U[i * n + i];  // Compute diagonal elements
        for (int j = i - 1; j >= 0; --j)
        {
            Real sum = 0.0;
            for (unsigned int k = j + 1; k < n; ++k)
            {
                sum += U[j * n + k] * inv_U[k * n + i];
            }
            inv_U[j * n + i] = -sum / U[j * n + j];
        }
    }
}

template <typename Real>
__global__ void upper_triangular_inverse_kernel(Real* U, unsigned int n, Real* inv_U)
{
    upper_triangular_inverse<Real>(U, n, inv_U);
}

template <typename Real>
void upper_triangular_inverse_cuda(Real* U, unsigned int n, Real* inv_U)
{
    upper_triangular_inverse_kernel<Real> << <1, 1 >> > (U, n, inv_U);
}