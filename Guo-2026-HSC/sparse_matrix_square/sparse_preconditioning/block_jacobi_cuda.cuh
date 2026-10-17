#pragma once
#include "diagonal_accumulation_cuda.cuh"
#include "math/gauss_jordan_cuda.cuh"

template<typename Real, unsigned int block_size_m>
__global__ void diagonal_inverse_inplace_kernel(Real* nonzeros_diagonal_m, unsigned int mat_size)
{
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= nBlocks_m) return;
    if (nBlocks_m == 0u || mat_size / block_size_m == 0u)
    {
        printf("diagonal_inverse_inplace_kernel::ERROR: nBlocks_m == 0u\n");
        return;
    }
    unsigned int block_size_m_current = (i < nBlocks_m - 1u || mat_size % block_size_m == 0) ? block_size_m : mat_size % block_size_m;
    Real mat[block_size_m * block_size_m];
    //printf("i = %d, block_size_m_current = %d\n", i, block_size_m_current);
    memcpy(mat, &nonzeros_diagonal_m[block_size_m * block_size_m * i], sizeof(Real) * block_size_m_current * block_size_m_current);
    bool success = gauss_jordan_inverse<Real>(mat, &nonzeros_diagonal_m[block_size_m * block_size_m * i], block_size_m_current);
    if (success == false)
    {
        printf("block_size_m = %d, i = %d, block_size_m_current = %d", block_size_m, i, block_size_m_current);
    }
}

template<typename Real, unsigned int block_size_m>
__global__ void block_dmv_kernel(Real* nonzeros_diagonal_m, Real* vec, Real* result, unsigned int mat_size)
{
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= nBlocks_m) return;
    if (nBlocks_m == 0u || mat_size / block_size_m == 0u)
    {
        printf("diagonal_inverse_inplace_kernel::ERROR: nBlocks_m == 0u\n");
        return;
    }
    unsigned int block_size_m_current = (i < nBlocks_m - 1u || mat_size % block_size_m == 0) ? block_size_m : mat_size % block_size_m;
    matVecMul<Real>(&result[block_size_m * i],
        &nonzeros_diagonal_m[block_size_m * block_size_m * i],
        &vec[block_size_m * i],
        block_size_m_current, block_size_m_current);
}

template<typename Real, unsigned int block_size_m>
void diagonal_inverse_inplace(Real* nonzeros_diagonal_m, unsigned int mat_size)
{
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    //printf("nBlocks_m = %d\n", nBlocks_m);
    diagonal_inverse_inplace_kernel<Real, block_size_m> << <get_blocksPerGrid(nBlocks_m), THREADS_PER_BLOCK >> > (nonzeros_diagonal_m, mat_size);
}

template<typename Real, unsigned int block_size_m>
inline void block_dmv(Real* nonzeros_diagonal_m, Real* vec, Real* result, unsigned int mat_size)
{
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    block_dmv_kernel<Real, block_size_m> << <get_blocksPerGrid(nBlocks_m), THREADS_PER_BLOCK >> > (nonzeros_diagonal_m, vec, result, mat_size);
}