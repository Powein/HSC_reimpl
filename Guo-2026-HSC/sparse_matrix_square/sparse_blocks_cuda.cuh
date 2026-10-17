#pragma once
#include <cuda_runtime.h>
#include "sparse_matrix_square/sparse_matrix_cuda.cuh"

// square-only
// no atomicAdd involved when doing assembly
template<typename Real, unsigned int block_size>
class sparse_blocks_cuda
{
public:
    sparse_blocks_cuda(unsigned int nBlocks_buffer, bool isSymmetric = false)
    {
        if (isSymmetric == true) m_type = 1u;
        m_nBlocks_buffer = nBlocks_buffer;
        m_nBlocks = nBlocks_buffer;
        cudaMalloc((void**)&dev_coords, sizeof(unsigned int) * nBlocks_buffer * 2);
        cudaMalloc((void**)&dev_nonzeros, sizeof(Real) * nBlocks_buffer * block_size * block_size);
    }
    void multiply_coefficient(Real coeff)
    {
        unsigned int N;
        N = m_nBlocks * block_size * block_size;
        scale_matrix_kernel<Real> << <get_blocksPerGrid(N), THREADS_PER_BLOCK >> > (N, coeff, dev_nonzeros);
    }
    ~sparse_blocks_cuda()
    {
        if (dev_coords) cudaFree(dev_coords);
        if (dev_nonzeros) cudaFree(dev_nonzeros);
    }
    void cuda_memset_zero()
    {
        cudaMemset(dev_nonzeros, 0, sizeof(Real) * m_nBlocks_buffer * block_size * block_size);
        this->m_nBlocks = 0u;
    }
    unsigned int* dev_coords = nullptr;
    Real* dev_nonzeros = nullptr;
    unsigned int m_nBlocks;
    unsigned int m_nBlocks_buffer;
    unsigned int m_type = 0u; // 0: base; 1: symmetric;
};