#pragma once
#include <cuda_runtime.h>
#include <vector>
#include <iostream>
#include "utils/threads_per_block.cuh"

template<typename Real>
__global__ void scale_matrix_kernel(unsigned int N, Real coeff, Real* nonzeros)
{
    unsigned int n = blockDim.x * blockIdx.x + threadIdx.x;
    if (n >= N) return;
    nonzeros[n] *= coeff;
}

// square-only
template<typename Real, unsigned int block_size>
class sparse_matrix_cuda
{
public:
    sparse_matrix_cuda() {}

    sparse_matrix_cuda(unsigned int dim, unsigned int nnz)
    {
        m_dim = dim;
        m_nnz = nnz;
        cuda_mem_alloc();
    }
    sparse_matrix_cuda(std::vector<unsigned int>& rows_to_cols, std::vector<unsigned int>& offsets)
    {
        m_dim = offsets.size() - 1;
        m_nnz = rows_to_cols.size();
        cuda_mem_alloc();
        cudaMemcpy(dev_rows_to_cols, rows_to_cols.data(), sizeof(unsigned int) * m_nnz, cudaMemcpyHostToDevice);
        cudaMemcpy(dev_offsets, offsets.data(), sizeof(unsigned int) * (m_dim + 1), cudaMemcpyHostToDevice);
        get_blocks_to_coords(rows_to_cols.data(), offsets.data());
    }
    sparse_matrix_cuda(unsigned int dim, unsigned int nnz, unsigned int* rows_to_cols, unsigned int* offsets)
    {
        m_dim = dim;
        m_nnz = nnz;
        cuda_mem_alloc();
        cudaMemcpy(dev_rows_to_cols, rows_to_cols, sizeof(unsigned int) * m_nnz, cudaMemcpyHostToDevice);
        cudaMemcpy(dev_offsets, offsets, sizeof(unsigned int) * (m_dim + 1), cudaMemcpyHostToDevice);
        get_blocks_to_coords(rows_to_cols, offsets);
    }
    virtual void cuda_memset_zero()
    {
        cudaMemset(dev_nonzeros, 0, sizeof(Real) * m_nnz * block_size * block_size);
    }
    virtual void multiply_coefficient(Real coeff)
    {
        unsigned int N = m_nnz * block_size * block_size;
        scale_matrix_kernel<Real> << <get_blocksPerGrid(N), THREADS_PER_BLOCK >> > (N, coeff, dev_nonzeros);
    }
    ~sparse_matrix_cuda()
    {
        if (this->dev_nonzeros) cudaFree(this->dev_nonzeros);
        if (this->dev_rows_to_cols) cudaFree(this->dev_rows_to_cols);
        if (this->dev_blocks_to_coords) cudaFree(this->dev_blocks_to_coords);
        if (this->dev_offsets) cudaFree(this->dev_offsets);
    }
    Real* dev_nonzeros = nullptr;
    unsigned int* dev_rows_to_cols = nullptr;
    unsigned int* dev_offsets = nullptr; // size = m_dim + 1
    unsigned int m_dim; // #Dims of blocks: 3N x 3N -> m_dim = N
    unsigned int m_nnz;
    unsigned int m_type = 0u; // 0: base; 1: symmetric;
    unsigned int* dev_blocks_to_coords = nullptr;
protected:
    virtual void get_blocks_to_coords(unsigned int* rows_to_cols, unsigned int* offsets)
    {
        unsigned int* blocks_to_coords = new unsigned int[m_nnz * 2];
        for (unsigned int i = 0; i < m_dim; ++i)
        {
            for (unsigned int j = offsets[i]; j < offsets[i + 1]; ++j)
            {
                blocks_to_coords[2 * j] = i;
                blocks_to_coords[2 * j + 1] = rows_to_cols[j];
            }
        }
        cudaMalloc((void**)&dev_blocks_to_coords, sizeof(unsigned int) * m_nnz * 2);
        cudaMemcpy(dev_blocks_to_coords, blocks_to_coords, sizeof(unsigned int) * m_nnz * 2, cudaMemcpyHostToDevice);
        delete[] blocks_to_coords;
    }
    virtual void cuda_mem_alloc()
    {
        // std::cout << "sparse_matrix_cuda::cuda_mem_alloc\n";
        cudaMalloc((void**)&dev_rows_to_cols, sizeof(unsigned int) * m_nnz);
        cudaMalloc((void**)&dev_offsets, sizeof(unsigned int) * (m_dim + 1));
        cudaMalloc((void**)&dev_nonzeros, sizeof(Real) * m_nnz * block_size * block_size);
    }
};