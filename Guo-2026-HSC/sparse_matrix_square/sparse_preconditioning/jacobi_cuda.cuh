#pragma once
#include <cuda_runtime.h>
#include "sparse_matrix_square/sparse_assembly_cuda.cuh"
#include "math/math_cuda.cuh"

// block inverse for 2x2, 3x3, and diagonal inverse for other block size
template<typename Real, unsigned int block_size>
__global__ void diagonal_inverse_inplace_kernel(unsigned int dim, Real* nonzeros_diagonal)
{
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= dim) return;
    if (block_size == 2)
    {
        Real inv2[4];
        Real d;
        det2<Real>(d, &nonzeros_diagonal[4 * i]);
        if (d < EPSILON)
        {
            memset(inv2, 0, sizeof(Real) * 4);
            inv2[0] = 1.;
            inv2[3] = 1.;
        }
        else
        {
            matInv2<Real>(inv2, &nonzeros_diagonal[4 * i]);
        }
        for (unsigned int j = 0; j < 4; ++j)
        {
            nonzeros_diagonal[4 * i + j] = inv2[j];
        }
    }
    else if (block_size == 3)
    {
        Real inv3[9];
        Real d;
        det3<Real>(d, &nonzeros_diagonal[9 * i]);
        if (d < EPSILON)
        {
            memset(inv3, 0, sizeof(Real) * 9);
            inv3[0] = 1.;
            inv3[4] = 1.;
            inv3[8] = 1.;
            //printf("diagonal_inverse_inplace_kernel::d < EPSILON\n");
        }
        else
        {
            matInv3<Real>(inv3, &nonzeros_diagonal[9 * i]);
        }
        for (unsigned int j = 0; j < 9; ++j)
        {
            nonzeros_diagonal[9 * i + j] = inv3[j];
        }
    }
    else
    {
        unsigned int offset = block_size * block_size * i;
        for (unsigned int j = 0; j < block_size; ++j) for (unsigned int k = 0; k < block_size; ++k)
        {
            if (j == k)
            {
                if (nonzeros_diagonal[offset + block_size * j + k] < EPSILON)
                {
                    nonzeros_diagonal[offset + block_size * j + k] = 1.;
                }
                else
                {
                    nonzeros_diagonal[offset + block_size * j + k] = 1. / nonzeros_diagonal[offset + block_size * j + k];
                }
            }
            else
            {
                nonzeros_diagonal[offset + block_size * j + k] = 0.;
            }
        }
    }
}

template<typename Real, unsigned int block_size>
__global__ void diagonal_inverse_kernel(unsigned int dim, Real* nonzeros_diagonal, Real* nonzeros_diagonal_inverse)
{
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= dim) return;
    if (block_size == 1)
    {
        nonzeros_diagonal_inverse[i] = 1. / nonzeros_diagonal[i];
    }
    if (block_size == 2)
    {
        matInv2<Real>(&nonzeros_diagonal_inverse[4 * i], &nonzeros_diagonal[4 * i]);
    }
    else if (block_size == 3)
    {
        matInv3<Real>(&nonzeros_diagonal_inverse[9 * i], &nonzeros_diagonal[9 * i]);
    }
    else
    {
        unsigned int offset = block_size * block_size * i;
        for (unsigned int j = 0; j < block_size; ++j) for (unsigned int k = 0; k < block_size; ++k)
        {
            if (j == k)
            {
                nonzeros_diagonal_inverse[offset + block_size * j + k] = 1. / nonzeros_diagonal[offset + block_size * j + k];
            }
            else
            {
                nonzeros_diagonal_inverse[offset + block_size * j + k] = 0.;
            }
        }
    }
}

template<typename Real, unsigned int block_size>
__global__ void diagonal_accumulation_kernel(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* sb_nonzeros, Real* nonzeros_diagonal)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nBlocks) return;
    if (blocks_to_coords[2 * b] != blocks_to_coords[2 * b + 1]) return;
    unsigned int coo = blocks_to_coords[2 * b];
    unsigned int bs_sq = block_size * block_size;
    for (unsigned int i = 0; i < bs_sq; ++i)
    {
        atomicAdd(&nonzeros_diagonal[coo * bs_sq + i], sb_nonzeros[b * bs_sq + i]);
    }
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_blocks_cuda<Real, block_size>& mat, Real* nonzeros_diagonal, bool memset_zero = true)
{
    if (dim == 0) return;
    if (memset_zero)
    {
        cudaMemset(nonzeros_diagonal, 0, sizeof(Real) * dim * block_size * block_size);
    }
    diagonal_accumulation_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, nonzeros_diagonal);
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_blocks_assembly_cuda<Real, block_size>& mat_assembly, Real* nonzeros_diagonal, bool memset_zero = true)
{
    if (dim == 0) return;
    if (memset_zero)
    {
        cudaMemset(nonzeros_diagonal, 0, sizeof(Real) * dim * block_size * block_size);
    }
    for (unsigned int i = 0; i < mat_assembly.dev_sparse_blocks.size(); ++i)
    {
        auto& mat = mat_assembly.dev_sparse_blocks[i];
        if (mat->m_nBlocks > 0)
        {
            diagonal_accumulation<Real, block_size>(dim, *mat, nonzeros_diagonal, false);
        }
    }
}

template <typename Real, unsigned int block_size>
inline void diagonal_accumulation(unsigned int dim, sparse_assembly_cuda<Real, block_size>& mat, Real* nonzeros_diagonal, bool memset_zero = true)
{
    if (dim == 0) return;
    if (mat.dev_sm != nullptr || (mat.dev_sb != nullptr && mat.dev_sb->m_type == 0u))
    {
        std::cout << "ERROR::diagonal_accumulation::Asymmetric matrix.\n";
        exit(1);
    }
    if (memset_zero)
    {
        cudaMemset(nonzeros_diagonal, 0, sizeof(Real) * dim * block_size * block_size);
    }
    if (mat.dev_sms != nullptr)
    {
        cudaMemcpy(nonzeros_diagonal, mat.dev_sms->dev_nonzeros_diagonal, sizeof(Real) * dim * block_size * block_size, cudaMemcpyDeviceToDevice);
    }
    else
    {
        std::cout << "ERROR::diagonal_accumulation::ill-conditioned (not full-rank).\n";
        exit(1);
    }
    if (mat.dev_sb != nullptr && mat.dev_sb->m_nBlocks > 0)
    {
        diagonal_accumulation<Real, block_size>(dim, *mat.dev_sb, nonzeros_diagonal, false);
    }
}

template <typename Real, unsigned int block_size>
inline void diagonal_inverse_inplace(unsigned int dim, Real* nonzeros_diagonal)
{
    diagonal_inverse_inplace_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, nonzeros_diagonal);
}

template <typename Real, unsigned int block_size>
inline void diagonal_inverse(unsigned int dim, Real* nonzeros_diagonal, Real* nonzeros_diagonal_inverse)
{
    diagonal_inverse_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, nonzeros_diagonal, nonzeros_diagonal_inverse);
}

template<typename Real, unsigned int block_size>
__global__ void dmv_kernel(unsigned int dim, Real* block_diagonals, Real* vec, Real* result)
{
    unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i >= dim) return;
    unsigned int bs2 = block_size * block_size;
    if (block_size == 1)
    {
        result[i] = block_diagonals[i] * vec[i];
    }
    else
    {
        matVecMul<Real>(&result[block_size * i], &block_diagonals[bs2 * i], &vec[block_size * i], block_size, block_size);
    }
}

template<typename Real, unsigned int block_size>
inline void compute_jacobi_preconditioner(sparse_blocks_cuda<Real, block_size>& mat, unsigned int dim, Real* jacobi_preconditioner)
{
    if (mat.m_type == 0u)
    {
        std::cout << "ERROR::compute_jacobi_preconditioner::Asymmetric matrix.\n";
        exit(1);
    }
    diagonal_accumulation<Real, block_size>(dim, mat, jacobi_preconditioner);
    diagonal_inverse_inplace<Real, block_size>(dim, jacobi_preconditioner);
}

template<typename Real, unsigned int block_size>
inline void compute_jacobi_preconditioner(sparse_matrix_symmetric_cuda<Real, block_size>& mat, unsigned int dim, Real* jacobi_preconditioner)
{
    cudaMemcpy(jacobi_preconditioner, mat.dev_nonzeros_diagonal, sizeof(Real) * dim * block_size * block_size, cudaMemcpyDeviceToDevice);
    diagonal_inverse_inplace<Real, block_size>(dim, jacobi_preconditioner);
}

template<typename Real, unsigned int block_size>
inline void compute_jacobi_preconditioner(sparse_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* jacobi_preconditioner)
{
    diagonal_accumulation<Real, block_size>(dim, mat, jacobi_preconditioner);
    diagonal_inverse_inplace<Real, block_size>(dim, jacobi_preconditioner);
}

template<typename Real, unsigned int block_size>
inline void compute_jacobi_preconditioner(sparse_blocks_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* jacobi_preconditioner)
{
    diagonal_accumulation<Real, block_size>(dim, mat, jacobi_preconditioner);
    //print_debug<Real>(block_size * block_size * dim, jacobi_preconditioner);
    diagonal_inverse_inplace<Real, block_size>(dim, jacobi_preconditioner);
}

// diagonal matrix vector multiplication
template<typename Real, unsigned int block_size>
inline void dmv(unsigned int dim, Real* block_diag_mat, Real* vec, Real* result)
{
    dmv_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, block_diag_mat, vec, result);
}