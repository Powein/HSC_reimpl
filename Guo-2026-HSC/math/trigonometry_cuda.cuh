#pragma once
#include <cuda_runtime.h>
#include "math_cuda.cuh"

template <typename Real>
__host__ __device__ __forceinline__
Real sin_tri_cuda(Real* x0, Real* x1, Real* x2)
{
    Real vec1[3];
    Real vec2[3];
    
    vecSubs3<Real>(vec1, x1, x0);
    vecSubs3<Real>(vec2, x2, x0);

    Real cross_vec[3];
    cross3<Real>(cross_vec, vec1, vec2);

    Real cross_norm = len3<Real>(cross_vec);
    
    Real norm1 = len3<Real>(vec1);
    Real norm2 = len3<Real>(vec2);
    
    if (norm1 == Real(0) || norm2 == Real(0))
    {
        return Real(0);
    }
    
    return cross_norm / (norm1 * norm2);
}

template <typename Real>
__host__ __device__ __forceinline__
Real cos_tri_cuda(Real* x0, Real* x1, Real* x2)
{
    Real vec1[3];
    Real vec2[3];

    vecSubs3<Real>(vec1, x1, x0);
    vecSubs3<Real>(vec2, x2, x0);

    Real d = dot3<Real>(vec1, vec2);
    
    Real norm1 = len3<Real>(vec1);
    Real norm2 = len3<Real>(vec2);
    
    if (norm1 == Real(0) || norm2 == Real(0))
    {
        return Real(1);
    }
    
    return d / (norm1 * norm2);
}

template <typename Real>
__host__ __device__ __forceinline__
Real tan_tri_cuda(Real* x0, Real* x1, Real* x2)
{
    Real sin_val = sin_tri_cuda<Real>(x0, x1, x2);
    Real cos_val = cos_tri_cuda<Real>(x0, x1, x2);

    if (cos_val == Real(0))
    {
        return sin_val > Real(0) ? FLT_MAX : -FLT_MAX;
    }

    return sin_val / cos_val;
}

template <typename Real>
__host__ __device__ __forceinline__
Real cot_tri_cuda(Real* x0, Real* x1, Real* x2)
{
    Real sin_val = sin_tri_cuda<Real>(x0, x1, x2);
    Real cos_val = cos_tri_cuda<Real>(x0, x1, x2);

    if (sin_val == Real(0))
    {
        return cos_val > Real(0) ? FLT_MAX : -FLT_MAX;
    }

    return cos_val / sin_val;
}