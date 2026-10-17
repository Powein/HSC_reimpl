#pragma once
#include <memory>
#include "sparse_matrix_symmetric_cuda.cuh"
#include "sparse_blocks_cuda.cuh"
#include "sparse_blocks_nonsquare_cuda.cuh"
#include "spmv_cuda.cuh"

template <typename Real>
struct sparse_assembly_fem_abd_cuda
{
    std::shared_ptr<sparse_matrix_symmetric_cuda<Real, 3>> dev_sms_fem = nullptr;
    std::shared_ptr<sparse_blocks_cuda<Real, 3>> dev_sb_fem = nullptr;
    std::shared_ptr<sparse_blocks_cuda<Real, 12>> dev_sb_abd = nullptr;
    std::shared_ptr<sparse_blocks_nonsquare_cuda<Real, 12, 3>> dev_sb_coupling = nullptr;
};

template <typename Real>
void multiply_coefficient(Real coeff, sparse_assembly_fem_abd_cuda<Real> mat)
{
    if (mat.dev_sms_fem != nullptr) mat.dev_sms_fem->multiply_coefficient(coeff);
    if (mat.dev_sb_fem != nullptr) mat.dev_sb_fem->multiply_coefficient(coeff);
    if (mat.dev_sb_abd != nullptr) mat.dev_sb_abd->multiply_coefficient(coeff);
    if (mat.dev_sb_coupling != nullptr) mat.dev_sb_coupling->multiply_coefficient(coeff);
}

template <typename Real>
void spmv_cuda(sparse_assembly_fem_abd_cuda<Real>& mat, unsigned int dim_fem, unsigned int dim_abd, Real* vec_fem, Real* vec_abd,
    Real* result_fem, Real* result_abd, bool isMemsetZero = true)
{
    std::shared_ptr<sparse_matrix_symmetric_cuda<Real, 3>>& sms_fem3 = mat.dev_sms_fem;
    std::shared_ptr<sparse_blocks_cuda<Real, 3>>& sb_fem3 = mat.dev_sb_fem;
    std::shared_ptr<sparse_blocks_cuda<Real, 12>>& sb_abd12 = mat.dev_sb_abd;
    std::shared_ptr<sparse_blocks_nonsquare_cuda<Real, 12, 3>>& sb_coupling12x3 = mat.dev_sb_coupling;

    if (isMemsetZero == true)
    {
        cudaMemset(result_fem, 0, sizeof(Real) * dim_fem * 3);
        cudaMemset(result_abd, 0, sizeof(Real) * dim_abd * 12);
    }

    if (sms_fem3 != nullptr) spmv_cuda<Real, 3>(*sms_fem3, dim_fem, vec_fem, result_fem, false);
    if (sb_fem3 != nullptr) spmv_cuda<Real, 3>(*sb_fem3, dim_fem, vec_fem, result_fem, false);
    if (sb_abd12 != nullptr) spmv_cuda<Real, 12>(*sb_abd12, dim_abd, vec_abd, result_abd, false);

    if (sb_coupling12x3 != nullptr)
    {
        spmv_cuda_ignore_offset<Real, 12, 3>(*sb_coupling12x3, dim_fem * 3, vec_fem, result_abd, false, false);
        spmv_cuda_ignore_offset<Real, 12, 3>(*sb_coupling12x3, dim_abd * 12, vec_abd, result_fem, false, true);
    }
}
