#pragma once
#include <cuda_runtime.h>
#include "math/math_cuda.cuh"
#include "utils/threads_per_block.cuh"
#include "utils/print_debug_cuda.cuh"

template <typename Real>
__global__ void find_delta_max_kernel(unsigned int nVerts, Real* positions_delta, Real* delta_max)
{
    unsigned int v = blockDim.x * blockIdx.x + threadIdx.x;
    if (v >= nVerts) return;
    Real delta = len3<Real>(&positions_delta[3 * v]);
    delta_max[0] = max<Real>(delta, delta_max[0]);
}

template <typename Real>
__global__ void find_delta_min_kernel(unsigned int nVerts, Real* positions_delta, Real* delta_min)
{
    unsigned int v = blockDim.x * blockIdx.x + threadIdx.x;
    if (v >= nVerts) return;
    Real delta = len3<Real>(&positions_delta[3 * v]);
    delta_min[0] = max<Real>(delta, delta_min[0]);
}

template <typename Real>
inline Real find_delta_max(Real nVerts, Real* positions_delta)
{
    Real* delta_max;
    cudaMalloc((void**)&delta_max, sizeof(Real));
    cudaMemset(delta_max, 0, sizeof(Real));
    find_delta_max_kernel<Real><< <get_blocksPerGrid(nVerts), THREADS_PER_BLOCK >> >(nVerts, positions_delta, delta_max);
    Real h_delta_max = visit_device<Real>(delta_max, 0);
    cudaFree(delta_max);
    return h_delta_max;
}

template <typename Real>
inline Real find_delta_min(Real nVerts, Real* positions_delta)
{
    Real* delta_min;
    cudaMalloc((void**)&delta_min, sizeof(Real));
    cudaMemset(delta_min, 0, sizeof(Real));
    find_delta_min_kernel<Real><< <get_blocksPerGrid(nVerts), THREADS_PER_BLOCK >> >(nVerts, positions_delta, delta_min);
    Real h_delta_min = visit_device<Real>(delta_min, 0);
    cudaFree(delta_min);
    return h_delta_min;
}