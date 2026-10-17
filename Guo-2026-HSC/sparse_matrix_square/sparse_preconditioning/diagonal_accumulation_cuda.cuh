#pragma once
#include "math/gauss_jordan_cuda.cuh"
#include "sparse_matrix_square/sparse_assembly_cuda.cuh"
#include <cuda_runtime.h>
// Jacobi preconditioner with user-defined block size

template<typename Real, unsigned int block_size> // the preconditioner's block size
__global__ void diagonal_accumulation_kernel(unsigned int nBlocks, unsigned int* blocks_to_coords,
    Real* sb_nonzeros, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool is_symmetric = true)
    // nonzeros_diagonal_m structure: NxN, NxN, NxN, nxn (the remainder)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nBlocks) return;
    //printf("%d\n", b);
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation_kernel::ERROR: preconditioner block size not matched!\n");
        return;
    }

    unsigned int block_i = blocks_to_coords[2 * b];
    unsigned int block_j = blocks_to_coords[2 * b + 1];
    unsigned int ratio = block_size_m / block_size; // i.e. 32
    //printf("ratio = %d\n", ratio);

    if (block_i / ratio != block_j / ratio)
    {
        // not in the same preconditioner's block
        return;
    }

    // preconditioner assembly
    unsigned int bId_m = block_i / ratio;
    unsigned int offset_x = block_i % ratio;
    unsigned int offset_y = block_j % ratio;

    unsigned int start_m = bId_m * block_size_m * block_size_m;
    unsigned int start_b = b * block_size * block_size;
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    if (nBlocks_m == 0u || mat_size / block_size_m == 0u)
    {
        printf("diagonal_accumulation_kernel::ERROR: nBlocks_m == 0u\n");
        return;
    }

    // the last block
    unsigned int block_size_m_current = (bId_m < nBlocks_m - 1u || mat_size % block_size_m == 0) ? block_size_m : mat_size % block_size_m;
    // M(offset_x, offset_y) = b

    //printf("block_i = %d, block_j = %d, bId_m = %d, offset_x = %d, offset_y = %d | %f %f %f %f %f %f %f %f %f\n",
    //    block_i, block_j, bId_m, offset_x, offset_y,
    //    sb_nonzeros[start_b], sb_nonzeros[start_b + 1], sb_nonzeros[start_b + 2], sb_nonzeros[start_b + 3], sb_nonzeros[start_b + 4],
    //    sb_nonzeros[start_b + 5], sb_nonzeros[start_b + 6], sb_nonzeros[start_b + 7], sb_nonzeros[start_b + 8]);
    //if (bId_m != 3u) return;
    //printf("bId_m = %d, nBlocks_m = %d, block_size_m_current = %d\n", bId_m, nBlocks_m, block_size_m_current);
    for (unsigned int i = 0; i < block_size; ++i) for (unsigned int j = 0; j < block_size; ++j)
    {
        atomicAdd(&nonzeros_diagonals_m
            [
                start_m + // block start at
                    offset_x * block_size * block_size_m_current + offset_y * block_size + // inner block start at
                    block_size_m_current * i + j // offset
            ], sb_nonzeros[start_b + block_size * i + j]);
        // symmetric
        if (is_symmetric && offset_x != offset_y)
        {
            atomicAdd(&nonzeros_diagonals_m
                [
                    start_m + // block start at
                        offset_y * block_size * block_size_m_current + offset_x * block_size + // inner block start at
                        block_size_m_current * j + i // offset of the inner-most block
                ], sb_nonzeros[start_b + block_size * i + j]);
        }
    }
}

template<typename Real, unsigned int block_size> // the preconditioner's block size
__global__ void diagonal_accumulation_kernel(unsigned int dim, unsigned int* rows_to_cols, unsigned int* offsets,
    Real* nonzeros_lower, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool is_symmetric = true)
{
    unsigned int block_i = blockDim.x * blockIdx.x + threadIdx.x;
    if (block_i >= dim) return;
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation_kernel::ERROR: preconditioner block size not matched!\n");
        return;
    }
    unsigned int block_j;
    unsigned int ratio = block_size_m / block_size;
    for (unsigned int n = offsets[block_i]; n < offsets[block_i + 1]; ++n)
    {
        block_j = rows_to_cols[n];
        if (block_i / ratio != block_j / ratio)
        {
            // not in the same preconditioner's block
            continue;
        }
        unsigned int bId_m = block_i / ratio;
        unsigned int offset_x = block_i % ratio;
        unsigned int offset_y = block_j % ratio;
        unsigned int start_m = bId_m * block_size_m * block_size_m;
        unsigned int start_b = n * block_size * block_size;
        unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
        if (nBlocks_m == 0u || mat_size / block_size_m == 0u)
        {
            printf("diagonal_accumulation_kernel::ERROR: nBlocks_m == 0u\n");
            return;
        }
        unsigned int block_size_m_current = (bId_m < nBlocks_m - 1u || mat_size % block_size_m == 0) ? block_size_m : mat_size % block_size_m;
        for (unsigned int i = 0; i < block_size; ++i) for (unsigned int j = 0; j < block_size; ++j)
        {
            atomicAdd(&nonzeros_diagonals_m
                [
                    start_m + // block start at
                        offset_x * block_size * block_size_m_current + offset_y * block_size + // inner block start at
                        block_size_m_current * i + j // offset
                ], nonzeros_lower[start_b + block_size * i + j]);
            // symmetric
            if (is_symmetric && offset_x != offset_y)
            {
                atomicAdd(&nonzeros_diagonals_m
                    [
                        start_m + // block start at
                            offset_y * block_size * block_size_m_current + offset_x * block_size + // inner block start at
                            block_size_m_current * j + i // offset of the inner-most block
                    ], nonzeros_lower[start_b + block_size * i + j]);
            }
        }
    }
}

template<typename Real, unsigned int block_size> // the preconditioner's block size
__global__ void diagonal_accumulation_kernel(unsigned int dim, Real* nonzeros_diagonals, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size)
{
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= dim) return;
    //printf("%d\n", b);
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation_kernel::ERROR: preconditioner block size not matched!\n");
        return;
    }

    unsigned int block_i = i;
    unsigned int block_j = i;
    unsigned int ratio = block_size_m / block_size;

    // preconditioner assembly
    unsigned int bId_m = block_i / ratio;
    unsigned int offset_x = block_i % ratio;
    unsigned int offset_y = block_j % ratio;

    unsigned int start_m = bId_m * block_size_m * block_size_m;
    unsigned int start_b = i * block_size * block_size;
    unsigned int nBlocks_m = mat_size / block_size_m + (mat_size % block_size_m == 0 ? 0u : 1u);
    if (nBlocks_m == 0u || mat_size / block_size_m == 0u)
    {
        printf("diagonal_accumulation_kernel::ERROR: nBlocks_m == 0u\n");
        return;
    }

    unsigned int block_size_m_current = (bId_m < nBlocks_m - 1u || mat_size % block_size_m == 0) ? block_size_m : mat_size % block_size_m;
    for (unsigned int i = 0; i < block_size; ++i) for (unsigned int j = 0; j < block_size; ++j)
    {
        atomicAdd(&nonzeros_diagonals_m
            [
                start_m + // block start at
                    offset_x * block_size * block_size_m_current + offset_y * block_size + // inner block start at
                    block_size_m_current * i + j // offset
            ], nonzeros_diagonals[start_b + block_size * i + j]);
    }
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_blocks_cuda<Real, block_size>& mat, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool memset_zero = true)
{
    if (dim == 0) return;
    //printf("diagonal_accumulation::sparse_blocks_cuda\n");
    if (mat.m_type == 0u)
    {
        printf("diagonal_accumulation::ERROR: sparse_blocks_cuda asymmetric\n");
        exit(1);
    }
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation::ERROR: preconditioner block size not matched!\n");
        exit(1);
    }
    if (memset_zero)
    {
        const unsigned int mat_size_m = block_size * dim;
        unsigned int blocks = mat_size / block_size_m;
        unsigned int mods = mat_size_m % block_size_m;
        unsigned int nnz_m = blocks * block_size_m * block_size_m + mods * mods;
        cudaMemset(nonzeros_diagonals_m, 0, sizeof(Real) * nnz_m);
    }
    //printf("diagonal_accumulation, mat.m_nBlocks = %d\n", mat.m_nBlocks);
    diagonal_accumulation_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks,
        mat.dev_coords, mat.dev_nonzeros, nonzeros_diagonals_m, block_size_m, mat_size, mat.m_type);
}


template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_matrix_symmetric_cuda<Real, block_size>& mat, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool memset_zero = true)
{
    if (dim == 0) return;
    //printf("diagonal_accumulation::sparse_matrix_symmetric_cuda\n");
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation::ERROR: preconditioner block size not matched!\n");
        exit(1);
    }
    if (memset_zero)
    {
        const unsigned int mat_size_m = block_size * dim;
        unsigned int blocks = mat_size / block_size_m;
        unsigned int mods = mat_size_m % block_size_m;
        unsigned int nnz_m = blocks * block_size_m * block_size_m + mods * mods;
        cudaMemset(nonzeros_diagonals_m, 0, sizeof(Real) * nnz_m);
    }
    //printf("diagonal_accumulation, mat.m_nnz_lower = %d\n", mat.m_nnz_lower);
    diagonal_accumulation_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, mat.dev_rows_to_cols,
        mat.dev_offsets, mat.dev_nonzeros_lower, nonzeros_diagonals_m, block_size_m, mat_size, true);
    diagonal_accumulation_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim,
        mat.dev_nonzeros_diagonal, nonzeros_diagonals_m, block_size_m, mat_size);
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_matrix_cuda<Real, block_size>& mat, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool memset_zero = true)
{
    if (dim == 0) return;
    //printf("diagonal_accumulation::sparse_matrix_cuda\n");
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation::ERROR: preconditioner block size not matched!\n");
        exit(1);
    }
    if (memset_zero)
    {
        const unsigned int mat_size_m = block_size * dim;
        unsigned int blocks = mat_size / block_size_m;
        unsigned int mods = mat_size_m % block_size_m;
        unsigned int nnz_m = blocks * block_size_m * block_size_m + mods * mods;
        cudaMemset(nonzeros_diagonals_m, 0, sizeof(Real) * nnz_m);
    }
    //printf("diagonal_accumulation, mat.m_nnz_lower = %d\n", mat.m_nnz_lower);
    diagonal_accumulation_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, mat.dev_rows_to_cols,
        mat.dev_offsets, mat.dev_nonzeros, nonzeros_diagonals_m, block_size_m, mat_size, false);
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_assembly_cuda<Real, block_size>& mat, Real* nonzeros_diagonals_m, unsigned int block_size_m, unsigned int mat_size, bool memset_zero = true)
{
    if (dim == 0) return;
    if (block_size_m % block_size != 0)
    {
        printf("diagonal_accumulation::ERROR: preconditioner block size not matched!\n");
        exit(1);
    }
    if (memset_zero)
    {
        const unsigned int mat_size_m = block_size * dim;
        unsigned int blocks = mat_size / block_size_m;
        unsigned int mods = mat_size_m % block_size_m;
        unsigned int nnz_m = blocks * block_size_m * block_size_m + mods * mods;
        cudaMemset(nonzeros_diagonals_m, 0, sizeof(Real) * nnz_m);
    }
    if (mat.dev_sb != nullptr)
    {
        diagonal_accumulation<Real, block_size>(dim, *mat.dev_sb, nonzeros_diagonals_m, block_size_m, mat_size, false);
    }
    if (mat.dev_sms != nullptr)
    {
        diagonal_accumulation<Real, block_size>(dim, *mat.dev_sms, nonzeros_diagonals_m, block_size_m, mat_size, false);
    }
    if (mat.dev_sm != nullptr)
    {
        diagonal_accumulation<Real, block_size>(dim, *mat.dev_sm, nonzeros_diagonals_m, block_size_m, mat_size, false);
    }
}