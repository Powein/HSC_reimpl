#pragma once
#include <memory>
#include <vector>
#include "sparse_matrix_cuda.cuh"
#include "sparse_matrix_symmetric_cuda.cuh"
#include "sparse_blocks_cuda.cuh"

template<typename Real, unsigned int block_size>
struct sparse_assembly_cuda
{
    std::shared_ptr<sparse_matrix_cuda<Real, block_size>> dev_sm = nullptr;
    std::shared_ptr<sparse_matrix_symmetric_cuda<Real, block_size>> dev_sms = nullptr;
    std::shared_ptr<sparse_blocks_cuda<Real, block_size>> dev_sb = nullptr;
};

template<typename Real, unsigned int block_size>
void multiply_coefficient(Real coeff, sparse_assembly_cuda<Real, block_size> mat)
{
    if (mat.dev_sm != nullptr) mat.dev_sm->multiply_coefficient(coeff);
    if (mat.dev_sms != nullptr) mat.dev_sms->multiply_coefficient(coeff);
    if (mat.dev_sb != nullptr) mat.dev_sb->multiply_coefficient(coeff);
}

template<typename Real, unsigned int block_size>
struct sparse_blocks_assembly_cuda
{
    std::vector<std::shared_ptr<sparse_blocks_cuda<Real, block_size>>> dev_sparse_blocks;
};

template<typename Real, unsigned int block_size>
void multiply_coefficient(Real coeff, sparse_blocks_assembly_cuda<Real, block_size> mat)
{
    for (unsigned int i = 0; i < mat.dev_sparse_blocks.size(); ++i)
    {
        mat.dev_sparse_blocks[i]->multiply_coefficient(coeff);
    }
}
