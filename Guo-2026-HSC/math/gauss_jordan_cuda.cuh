#pragma once
#include <cuda_runtime.h>
#include <stdio.h>

template <typename Real>
__host__ __device__ __forceinline__
bool gauss_jordan_inverse(Real* matrix, Real* inverse, unsigned int mat_size)
{
    // Initialize the inverse matrix as an identity matrix
    for (unsigned int i = 0; i < mat_size; ++i)
    {
        for (unsigned int j = 0; j < mat_size; ++j)
        {
            inverse[i * mat_size + j] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (unsigned int i = 0; i < mat_size; ++i)
    {
        // If the diagonal element is zero, swap rows with non-zero elements
        if (matrix[i * mat_size + i] == 0.0)
        {
            unsigned int j;
            for (j = i + 1; j < mat_size && matrix[j * mat_size + i] == 0.0; ++j);
            if (j == mat_size)
            {
                printf("ERROR::gauss_jordan_inverse: Matrix is singular. Inverse does not exist.\n");
                for (unsigned int r = 0; r < mat_size; ++r)
                {
                    for (unsigned int c = 0; c < mat_size; ++c)
                    {
                        printf("%f ", matrix[mat_size * r + c]);
                    }
                    printf("\n");
                }
                return false;
            }
            // Swap rows i and j in both matrix and its inverse
            for (unsigned int k = 0; k < mat_size; ++k)
            {
                Real temp = matrix[i * mat_size + k];
                matrix[i * mat_size + k] = matrix[j * mat_size + k];
                matrix[j * mat_size + k] = temp;

                temp = inverse[i * mat_size + k];
                inverse[i * mat_size + k] = inverse[j * mat_size + k];
                inverse[j * mat_size + k] = temp;
            }
        }

        Real pivot = matrix[i * mat_size + i];
        // Scale the row to make the diagonal element 1
        for (unsigned int j = 0; j < mat_size; ++j)
        {
            matrix[i * mat_size + j] /= pivot;
            inverse[i * mat_size + j] /= pivot;
        }

        // Perform row operations to make other elements in the column zero
        for (unsigned int j = 0; j < mat_size; ++j)
        {
            if (i != j)
            {
                Real factor = matrix[j * mat_size + i];
                for (unsigned int k = 0; k < mat_size; ++k)
                {
                    matrix[j * mat_size + k] -= factor * matrix[i * mat_size + k];
                    inverse[j * mat_size + k] -= factor * inverse[i * mat_size + k];
                }
            }
        }
    }
    return true;
}

template <typename Real>
__global__ void gauss_jordan_inverse_kernel(Real* matrix, Real* inverse, unsigned int mat_size)
{
    bool GJ_success = false;
    GJ_success = gauss_jordan_inverse<Real>(matrix, inverse, mat_size);
    if (GJ_success == false)
    {
        printf("gauss_jordan_inverse_kernel<Real>::Error: matrix inverse failed!\n");
    }
}

// gauss-jordan: the skipping version