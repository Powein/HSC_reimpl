#pragma once
#include "spmv_cuda.cuh"

template<typename Real, unsigned int block_size>
__global__ void spmv_sm_offdiag_kernel(unsigned int nnz, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nnz) return;
	if (blocks_to_coords[2 * b] == blocks_to_coords[2 * b + 1]) return;
	Real mv[block_size];
	matVecMul<Real>(mv, &nonzeros[block_size * block_size * b], &vec[block_size * blocks_to_coords[2 * b + 1]], block_size, block_size);
	for (unsigned int i = 0; i < block_size; ++i)
	{
		atomicAdd(&result[block_size * blocks_to_coords[2 * b] + i], mv[i]);
	}
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sb_offdiag_kernel(unsigned int nBlocks, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nBlocks) return;
	if (blocks_to_coords[2 * b] == blocks_to_coords[2 * b + 1]) return;
	Real mv[block_size];
	matVecMul<Real>(mv, &nonzeros[block_size * block_size * b], &vec[block_size * blocks_to_coords[2 * b + 1]], block_size, block_size);
	for (unsigned int i = 0; i < block_size; ++i)
	{
		atomicAdd(&result[block_size * blocks_to_coords[2 * b] + i], mv[i]);
	}
}

template<typename Real, unsigned int block_size>
__global__ void spmv_sbs_offdiag_kernel(unsigned int nBlocks_diag_lower, unsigned int* blocks_to_coords, Real* nonzeros_diag_lower, Real* vec, Real* result)
{
	unsigned int b = blockDim.x * blockIdx.x + threadIdx.x;
	if (b >= nBlocks_diag_lower) return;
	if (blocks_to_coords[2 * b] == blocks_to_coords[2 * b + 1]) return;
	Real mv[block_size];
	matVecMul<Real>(mv, &nonzeros_diag_lower[block_size * block_size * b], &vec[block_size * blocks_to_coords[2 * b + 1]], block_size, block_size);
	for (unsigned int i = 0; i < block_size; ++i)
	{
		atomicAdd(&result[block_size * blocks_to_coords[2 * b] + i], mv[i]);
	}
	matTVecMul<Real>(mv, &nonzeros_diag_lower[block_size * block_size * b], &vec[block_size * blocks_to_coords[2 * b]], block_size, block_size);
	for (unsigned int i = 0; i < block_size; ++i)
	{
		atomicAdd(&result[block_size * blocks_to_coords[2 * b + 1] + i], mv[i]);
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_offdiag_cuda(sparse_matrix_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * mat.m_dim * block_size);
	spmv_sm_offdiag_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nnz), THREADS_PER_BLOCK >> > (mat.m_nnz, mat.dev_blocks_to_coords, mat.dev_nonzeros, vec, result);
}

template<typename Real, unsigned int block_size>
inline void spmv_offdiag_cuda(sparse_matrix_symmetric_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * mat.m_dim * block_size);
	spmv_sms_offdiagonal_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nnz_lower), THREADS_PER_BLOCK >> > (mat.m_nnz_lower, mat.dev_blocks_to_coords, mat.dev_nonzeros_lower, vec, result);
}

template<typename Real, unsigned int block_size>
inline void spmv_offdiag_cuda(sparse_blocks_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	if (mat.m_type == 0u)
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sb_offdiag_kernel<Real, block_size> << <get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
		}
	}
	else
	{
		if (mat.m_nBlocks > 0)
		{
			spmv_sbs_offdiag_kernel<Real, block_size> << < get_blocksPerGrid(mat.m_nBlocks), THREADS_PER_BLOCK >> > (mat.m_nBlocks, mat.dev_coords, mat.dev_nonzeros, vec, result);
		}
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_offdiag_cuda(sparse_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	if (mat.dev_sm != nullptr)
	{
		spmv_offdiag_cuda<Real, block_size>(*(mat.dev_sm), dim, vec, result, false);
	}
	if (mat.dev_sms != nullptr)
	{
		spmv_offdiag_cuda<Real, block_size>(*(mat.dev_sms), dim, vec, result, false);
	}
	if (mat.dev_sb != nullptr)
	{
		spmv_offdiag_cuda<Real, block_size>(*(mat.dev_sb), dim, vec, result, false);
	}
}

template<typename Real, unsigned int block_size>
inline void spmv_offdiag_cuda(sparse_blocks_assembly_cuda<Real, block_size>& mat, unsigned int dim, Real* vec, Real* result, bool isMemsetZero = true)
{
	if (isMemsetZero) cudaMemset(result, 0, sizeof(Real) * dim * block_size);
	for (unsigned int i = 0; i < mat.dev_sparse_blocks.size(); ++i)
	{
		spmv_offdiag_cuda<Real, block_size>(*(mat.dev_sparse_blocks[i]), dim, vec, result, false);
	}
}