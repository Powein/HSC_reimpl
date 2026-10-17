#pragma once
#include <cuda_runtime.h>
#include <stdio.h>
#include "utils/print_debug_cuda.cuh"

template <typename Real>
__host__ __device__ __forceinline__
bool cholesky_factorization(Real* A, Real* L, unsigned int mat_size)
{
    for (unsigned int i = 0; i < mat_size; ++i)
    {
        for (unsigned int j = 0; j <= i; ++j)
        {
            Real sum = 0.;

            if (j == i)
            {
                for (int k = 0; k < j; ++k)
                {
                    sum += L[mat_size * j + k] * L[mat_size * j + k];
                }
                if (A[mat_size * j + j] - sum <= 0)
                {
                    // Matrix is not positive definite
                    return false;
                }
                L[mat_size * j + j] = sqrt(A[mat_size * j + j] - sum);
            }
            else
            {
                // Off-diagonal elements
                for (unsigned int k = 0; k < j; ++k)
                {
                    sum += L[mat_size * i + k] * L[mat_size * j + k];
                }
                L[mat_size * i + j] = (A[mat_size * i + j] - sum) / L[mat_size * j + j];
            }
        }
    }
    return true;
}

// Function to solve L * y = b using forward substitution
template <typename Real>
__host__ __device__ __forceinline__
void forward_substitution(Real* L, Real* b, Real* y, unsigned int mat_size)
{
    for (unsigned int i = 0; i < mat_size; ++i)
    {
        Real sum = 0.;
        for (unsigned int j = 0; j < i; ++j)
        {
            sum += L[mat_size * i + j] * y[j];
        }
        y[i] = (b[i] - sum) / L[mat_size * i + i];
    }
}

// Function to solve L^T * x = y using backward substitution
template <typename Real>
__host__ __device__ __forceinline__
void backward_substitution(Real* L, Real* y, Real* x, unsigned int mat_size)
{
    for (int i = mat_size - 1; i >= 0; --i)
    {
        Real sum = 0.;
        for (int j = i + 1; j < mat_size; ++j)
        {
            sum += L[mat_size * j + i] * x[j];
        }
        x[i] = (y[i] - sum) / L[mat_size * i + i];
    }
}

template <typename Real>
__host__ __device__ __forceinline__
unsigned int solve_cholesky(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    Real* L = (Real*)malloc(mat_size * mat_size * sizeof(Real));
    // Perform Cholesky decomposition
    if (!cholesky_factorization<Real>(A, L, mat_size))
    {
        printf("Matrix is not positive definite.\n");
        //exit(EXIT_FAILURE);
        return 0u;
    }

    // Allocate space for intermediate vector y
    Real* y = (Real*)malloc(mat_size * sizeof(double));

    // Solve L * y = b
    forward_substitution<Real>(L, b, y, mat_size);

    // Solve L^T * x = y
    backward_substitution<Real>(L, y, x, mat_size);

    free(L);
    free(y);
    return 1u;
}

template<typename Real>
__global__ void solve_cholesky_kernel(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    solve_cholesky<Real>(A, b, x, mat_size);
}

template<typename Real>
__global__ void solve_cholesky_kernel(Real* A, Real* b, Real* x, unsigned int mat_size, unsigned int* succeed)
{
    succeed[0] = solve_cholesky<Real>(A, b, x, mat_size);
}

template <typename Real>
void solve_cholesky_cuda(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    solve_cholesky_kernel<Real> << <1, 1 >> > (A, b, x, mat_size);
}

template <typename Real>
unsigned int solve_cholesky_cuda2(Real* A, Real* b, Real* x, unsigned int mat_size)
{
    unsigned int* succeed;
    cudaMalloc((void**)&succeed, sizeof(unsigned int));
    solve_cholesky_kernel<Real> << <1, 1 >> > (A, b, x, mat_size, succeed);
    unsigned int succeed_bool = visit_device<unsigned int>(succeed, 0);
    cudaFree(succeed);
    return succeed_bool;
}