#pragma once
#include "sparse_matrix_cuda.cuh"
#include "utils/print_debug_cuda.cuh"

template<typename Real, unsigned int block_size>
class sparse_matrix_symmetric_cuda : public sparse_matrix_cuda<Real, block_size>
{
public:
    sparse_matrix_symmetric_cuda() {}

    sparse_matrix_symmetric_cuda(unsigned int dim, unsigned int nnz_lower)
    {
        this->m_dim = dim;
        m_nnz_lower = nnz_lower;
        this->m_nnz = this->m_dim + m_nnz_lower * 2;
        cuda_mem_alloc();
        this->m_type = 1u;
    }
    sparse_matrix_symmetric_cuda(std::vector<unsigned int>& rows_to_cols, std::vector<unsigned int>& offsets)
    {
        this->m_dim = offsets.size() - 1;
        m_nnz_lower = rows_to_cols.size();
        this->m_nnz = this->m_dim + m_nnz_lower * 2;
        cuda_mem_alloc();
        cudaMemcpy(this->dev_rows_to_cols, rows_to_cols.data(), sizeof(unsigned int) * m_nnz_lower, cudaMemcpyHostToDevice);
        cudaMemcpy(this->dev_offsets, offsets.data(), sizeof(unsigned int) * (this->m_dim + 1), cudaMemcpyHostToDevice);
        this->get_blocks_to_coords(rows_to_cols.data(), offsets.data());
        this->m_type = 1u;
        //print_debug<unsigned int>(m_nnz_lower * 2, dev_blocks_to_coords);
    }
    sparse_matrix_symmetric_cuda(unsigned int dim, unsigned int nnz_lower, unsigned int* rows_to_cols, unsigned int* offsets)
    {
        this->m_dim = dim;
        m_nnz_lower = nnz_lower;
        this->m_nnz = this->m_dim + m_nnz_lower * 2;
        cuda_mem_alloc();
        cudaMemcpy(this->dev_rows_to_cols, rows_to_cols, sizeof(unsigned int) * m_nnz_lower, cudaMemcpyHostToDevice);
        cudaMemcpy(this->dev_offsets, offsets, sizeof(unsigned int) * (this->m_dim + 1), cudaMemcpyHostToDevice);
        this->get_blocks_to_coords(rows_to_cols, offsets);
        this->m_type = 1u;
        //print_debug<unsigned int>(m_nnz_lower * 2, dev_blocks_to_coords);
    }
    virtual void cuda_memset_zero()
    {
        cudaMemset(dev_nonzeros_diagonal, 0, sizeof(Real) * this->m_dim * block_size * block_size);
        cudaMemset(dev_nonzeros_lower, 0, sizeof(Real) * m_nnz_lower * block_size * block_size);
    }
    virtual void multiply_coefficient(Real coeff)
    {
        unsigned int N;
        N = m_nnz_lower * block_size * block_size;
        scale_matrix_kernel<Real> << <get_blocksPerGrid(N), THREADS_PER_BLOCK >> > (N, coeff, dev_nonzeros_lower);
        N = this->m_dim * block_size * block_size;
        scale_matrix_kernel<Real> << <get_blocksPerGrid(N), THREADS_PER_BLOCK >> > (N, coeff, dev_nonzeros_diagonal);
    }
    ~sparse_matrix_symmetric_cuda()
    {
        if (this->dev_nonzeros_diagonal) cudaFree(this->dev_nonzeros_diagonal);
        if (this->dev_nonzeros_lower) cudaFree(this->dev_nonzeros_lower);
    }
    Real* dev_nonzeros_diagonal = nullptr;
    Real* dev_nonzeros_lower = nullptr;
    unsigned int m_nnz_lower;
protected:
    virtual void cuda_mem_alloc()
    {
        // std::cout << "sparse_matrix_symmetric_cuda::cuda_mem_alloc\n";
        //cudaMalloc((void**)&this->dev_nonzeros, sizeof(Real) * m_nnz * block_size * block_size);
        cudaMalloc((void**)&this->dev_rows_to_cols, sizeof(unsigned int) * m_nnz_lower);
        cudaMalloc((void**)&this->dev_offsets, sizeof(unsigned int) * (this->m_dim + 1));
        cudaMalloc((void**)&dev_nonzeros_diagonal, sizeof(Real) * this->m_dim * block_size * block_size);
        cudaMalloc((void**)&dev_nonzeros_lower, sizeof(Real) * m_nnz_lower * block_size * block_size);
    }
};