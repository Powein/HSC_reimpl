#pragma once
#include <cuda_runtime.h>
#include "utils/threads_per_block.cuh"

template<typename Real>
__global__ void inner_product_kernel(unsigned int N, Real* a, Real* b, Real* result)
{
    unsigned int index = threadIdx.x + blockIdx.x * blockDim.x;
    unsigned int stride = blockDim.x * gridDim.x;

    __shared__ Real cache[THREADS_PER_BLOCK]; // 256
    Real temp_sum = 0;
    for (unsigned int i = index; i < N; i += stride)
    {
        temp_sum += a[i] * b[i];
    }

    cache[threadIdx.x] = temp_sum;

    __syncthreads();
    
    unsigned int i = blockDim.x / 2;
    while (i != 0)
    {
        if (threadIdx.x < i)
        {
            cache[threadIdx.x] += cache[threadIdx.x + i];
        }
        __syncthreads();
        i /= 2;
    }

    if (threadIdx.x == 0)
    {
        atomicAdd(result, cache[0]);
    }
}
