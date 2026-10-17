#pragma once
#include "sparse_matrix_cuda.cuh"
#include "sparse_matrix_symmetric_cuda.cuh"
#include "sparse_blocks_cuda.cuh"
#include "sparse_blocks_nonsquare_cuda.cuh"
#include "sparse_assembly_cuda.cuh"
#include "utils/threads_per_block.cuh"
#include "math/math_cuda.cuh"

template<typename Real, unsigned int block_size>
__global__ void spmv_sm_kernel(unsigned int nnz, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
    unsigned int global_warp_id = (blockDim.x * blockIdx.x + threadIdx.x) / 32;
    if (global_warp_id >= nnz) return;

    auto lane_id = threadIdx.x % 32;
    unsigned int b = global_warp_id;

    unsigned int row_idx = blocks_to_coords[2 * b];
    unsigned int col_idx = blocks_to_coords[2 * b + 1];

    Real sum = 0;
    if (lane_id < block_size)
	{

        for (unsigned int j = 0; j < block_size; ++j)
		{	
            sum += nonzeros[b * block_size * block_size + lane_id * block_size + j] * vec[col_idx * block_size + j];
        }
        
        atomicAdd(&result[row_idx * block_size + lane_id], sum);
    }
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sms_offdiagonal_kernel(unsigned int nnz_lower, unsigned int* blocks_to_coords, Real* nonzeros_lower, Real* vec, Real* result)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nnz_lower) return;

    const unsigned int threads_per_block = 256; 
    
    __shared__ Real s_mv[threads_per_block * block_size];
    __shared__ Real s_mat[threads_per_block * block_size * block_size];

    Real* my_mv = &s_mv[threadIdx.x * block_size];
    Real* my_mat = &s_mat[threadIdx.x * block_size * block_size];

    unsigned int mat_offset = block_size * block_size * b;
    for(unsigned int i = 0; i < block_size * block_size; ++i) {
        my_mat[i] = nonzeros_lower[mat_offset + i];
    }

    unsigned int row_block = blocks_to_coords[2 * b];
    unsigned int col_block = blocks_to_coords[2 * b + 1];

    matVecMul<Real>(my_mv, my_mat, &vec[block_size * col_block], block_size, block_size);
    
    for (unsigned int i = 0; i < block_size; ++i)
    {
        atomicAdd(&result[block_size * row_block + i], my_mv[i]);
    }

    matTVecMul<Real>(my_mv, my_mat, &vec[block_size * row_block], block_size, block_size);
    
    for (unsigned int i = 0; i < block_size; ++i)
    {
        atomicAdd(&result[block_size * col_block + i], my_mv[i]);
    }
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sms_diagonal_kernel(unsigned int dim, Real* nonzeros_diagonal, Real* vec, Real* result)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= dim) return;

    const unsigned int threads_per_block = 256;
    __shared__ Real s_mv[threads_per_block * block_size];
    
    Real* my_mv = &s_mv[threadIdx.x * block_size];

    matVecMul<Real>(my_mv, &nonzeros_diagonal[block_size * block_size * b], &vec[block_size * b], block_size, block_size);

    unsigned int result_base = block_size * b;
    for (unsigned int i = 0; i < block_size; ++i)
    {
        result[result_base + i] += my_mv[i];
    }
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sb_kernel(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nBlocks) return;

    unsigned int row_block = blocks_to_coords[2 * b];
    unsigned int col_block = blocks_to_coords[2 * b + 1];

    Real mv[block_size];

    matVecMul<Real>(mv, &nonzeros[block_size * block_size * b], &vec[block_size * col_block], block_size, block_size);

    unsigned int result_base = block_size * row_block;
    #pragma unroll
    for (unsigned int i = 0; i < block_size; ++i)
    {
        atomicAdd(&result[result_base + i], mv[i]);
    }
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sbs_kernel(unsigned int nBlocks_diag_lower, unsigned int* blocks_to_coords, Real* nonzeros_diag_lower, Real* vec, Real* result)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nBlocks_diag_lower) return;

    const unsigned int threads_per_block = 256;
    __shared__ Real s_mv[threads_per_block * block_size];
    __shared__ Real s_mat[threads_per_block * block_size * block_size];

    Real* my_mv = &s_mv[threadIdx.x * block_size];
    Real* my_mat = &s_mat[threadIdx.x * block_size * block_size];

    unsigned int row_block = blocks_to_coords[2 * b];
    unsigned int col_block = blocks_to_coords[2 * b + 1];
    bool is_diagonal = (row_block == col_block);

    unsigned int mat_offset = block_size * block_size * b;
    for (unsigned int i = 0; i < block_size * block_size; ++i) {
        my_mat[i] = nonzeros_diag_lower[mat_offset + i];
    }

    matVecMul<Real>(my_mv, my_mat, &vec[block_size * col_block], block_size, block_size);
    
    unsigned int row_result_base = block_size * row_block;
    for (unsigned int i = 0; i < block_size; ++i)
    {
        atomicAdd(&result[row_result_base + i], my_mv[i]);
    }

    if (!is_diagonal)
    {
        matTVecMul<Real>(my_mv, my_mat, &vec[block_size * row_block], block_size, block_size);
        
        unsigned int col_result_base = block_size * col_block;
        for (unsigned int i = 0; i < block_size; ++i)
        {
            atomicAdd(&result[col_result_base + i], my_mv[i]);
        }
    }
}

template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
__global__ void spmv_sb_ns_kernel(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result,
    unsigned int offset_row = 0u, unsigned int offset_col = 0u)
{
    unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
    if (b >= nBlocks) return;

    const unsigned int threads_per_block = 256;
    __shared__ Real s_mv[threads_per_block * block_size_r];
    
    Real* my_mv = &s_mv[threadIdx.x * block_size_r];

    unsigned int row_block = blocks_to_coords[2 * b];
    unsigned int col_block = blocks_to_coords[2 * b + 1];

    unsigned int vec_base = block_size_c * col_block + offset_col;
    unsigned int result_base = block_size_r * row_block + offset_row;
    unsigned int mat_base = block_size_r * block_size_c * b;

    matVecMul<Real>(my_mv, &nonzeros[mat_base], &vec[vec_base], block_size_r, block_size_c);

    #pragma unroll
    for (unsigned int i = 0; i < block_size_r; ++i)
    {
        atomicAdd(&result[result_base + i], my_mv[i]);
    }
}

template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
__global__ void spmv_sbs_ns_kernel(unsigned int nBlocks_diag_lower, unsigned int* blocks_to_coords, Real* nonzeros_diag_lower, Real* vec, Real* result,
	unsigned int offset_row = 0u, unsigned int offset_col = 0u)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nBlocks_diag_lower) return;
	Real mv0[block_size_r];
	matVecMul<Real>(mv0, &nonzeros_diag_lower[block_size_r * block_size_c * b], &vec[block_size_c * blocks_to_coords[2 * b + 1] + offset_col], block_size_r, block_size_c);
	for (unsigned int i = 0; i < block_size_r; ++i)
	{
		atomicAdd(&result[block_size_r * blocks_to_coords[2 * b] + i + offset_row], mv0[i]);
	}
	Real mv1[block_size_c];
	matTVecMul<Real>(mv1, &nonzeros_diag_lower[block_size_r * block_size_c * b], &vec[block_size_r * blocks_to_coords[2 * b] + offset_row], block_size_c, block_size_r);
	for (unsigned int i = 0; i < block_size_c; ++i)
	{
		atomicAdd(&result[block_size_c * blocks_to_coords[2 * b + 1] + i + offset_col], mv1[i]);
	}
}

template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
__global__ void spmv_sb_ns_kernel_ignore_offset(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nBlocks) return;
	Real mv[block_size_r];
	matVecMul<Real>(mv, &nonzeros[block_size_r * block_size_c * b], &vec[block_size_c * blocks_to_coords[2 * b + 1]], block_size_r, block_size_c);
	for (unsigned int i = 0; i < block_size_r; ++i)
	{
		atomicAdd(&result[block_size_r * blocks_to_coords[2 * b] + i], mv[i]);
	}
}

template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
__global__ void spmTv_sb_ns_kernel_ignore_offset(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nBlocks) return;
	Real mv[block_size_c];
	matTVecMul<Real>(mv, &nonzeros[block_size_r * block_size_c * b], &vec[block_size_r * blocks_to_coords[2 * b]], block_size_c, block_size_r);
	for (unsigned int i = 0; i < block_size_c; ++i)
	{
		atomicAdd(&result[block_size_c * blocks_to_coords[2 * b + 1] + i], mv[i]);
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_cuda(sparse_matrix_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * mat.m_dim * block_size);
	spmv_sm_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nnz), THREADS_PER_BLOCK >> > (mat.m_nnz, mat.dev_blocks_to_coords, mat.dev_nonzeros, vec, result);
}

template<typename Real, unsigned int block_size>
inline void spmv_cuda(sparse_matrix_symmetric_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * mat.m_dim * block_size);
	spmv_sms_offdiagonal_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nnz_lower), THREADS_PER_BLOCK >> > (mat.m_nnz_lower, mat.dev_blocks_to_coords, mat.dev_nonzeros_lower, vec, result);
	spmv_sms_diagonal_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_dim), THREADS_PER_BLOCK >> > (mat.m_dim, mat.dev_nonzeros_diagonal, vec, result);
}

template<typename Real, unsigned int block_size>
inline void spmv_cuda(sparse_blocks_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	if (mat.m_type == 0u)
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sb_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
		}
	}
	else
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sbs_kernel<Real, block_size> << < get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
		}
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_cuda(sparse_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	if (mat.dev_sm != nullptr)
	{
		spmv_cuda<Real, block_size>(*(mat.dev_sm), dim, vec, result, false);
	}
	if (mat.dev_sms != nullptr)
	{
		spmv_cuda<Real, block_size>(*(mat.dev_sms), dim, vec, result, false);
	}
	if (mat.dev_sb != nullptr)
	{
		spmv_cuda<Real, block_size>(*(mat.dev_sb), dim, vec, result, false);
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_cuda(sparse_blocks_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	for (unsigned int i = 0; i < mat.dev_sparse_blocks.size(); ++i)
	{
		spmv_cuda<Real, block_size>(*(mat.dev_sparse_blocks[i]), dim, vec, result, false);
	}
}

// with offset
template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
inline void spmv_cuda(sparse_blocks_nonsquare_cuda<Real, block_size_r, block_size_c>& mat, unsigned int vec_len /* vector length !!!!! */,
	Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * vec_len);
	if (mat.m_type == 0u)
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sb_ns_kernel<Real, block_size_r, block_size_c> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (
				mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result, mat.m_offset_row, mat.m_offset_col);
		}
	}
	else
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sbs_ns_kernel<Real, block_size_r, block_size_c> << < get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (
				mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result, mat.m_offset_row, mat.m_offset_col);
		}
	}
}

// without offset
template<typename Real, unsigned int block_size_r, unsigned int block_size_c>
inline void spmv_cuda_ignore_offset(sparse_blocks_nonsquare_cuda<Real, block_size_r, block_size_c>& mat, unsigned int vec_len /* vector length !!!!! */,
	Real* vec, Real* result, bool isMemsetZero = true, bool transpose = false)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * vec_len);
	if (transpose == true)
	{
		spmTv_sb_ns_kernel_ignore_offset<Real, block_size_r, block_size_c> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (
			mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
	}
	else
	{
		spmv_sb_ns_kernel_ignore_offset<Real, block_size_r, block_size_c> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (
			mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
	}
}