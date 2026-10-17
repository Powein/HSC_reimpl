#pragma once
#include <cuda_runtime.h>
#include "math/math_cuda.cuh"

// General template declaration for atomicMin
template <typename Real>
__device__ Real atomicMin(Real* address, Real val);

// Specialization for float
template <>
__device__ float atomicMin(float* address, float val) {
    int* address_as_i = reinterpret_cast<int*>(address);
    int old = *address_as_i, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_i, assumed, __float_as_int(min<float>(val, __int_as_float(assumed))));
    } while (assumed != old);
    return __int_as_float(old);
}

// Specialization for double
template <>
__device__ double atomicMin(double* address, double val) {
    unsigned long long int* address_as_ull = reinterpret_cast<unsigned long long int*>(address);
    unsigned long long int old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(min<double>(val, __longlong_as_double(assumed))));
    } while (assumed != old);
    return __longlong_as_double(old);
}

// General template declaration for atomicMax
template <typename Real>
__device__ Real atomicMax(Real* address, Real val);

// Specialization for float
template <>
__device__ float atomicMax(float* address, float val) {
    int* address_as_i = reinterpret_cast<int*>(address);
    int old = *address_as_i, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_i, assumed, __float_as_int(max<float>(val, __int_as_float(assumed))));
    } while (assumed != old);
    return __int_as_float(old);
}

// Specialization for double
template <>
__device__ double atomicMax(double* address, double val) {
    unsigned long long int* address_as_ull = reinterpret_cast<unsigned long long int*>(address);
    unsigned long long int old = *address_as_ull, assumed;
    do {
        assumed = old;
        old = atomicCAS(address_as_ull, assumed, __double_as_longlong(max<double>(val, __longlong_as_double(assumed))));
    } while (assumed != old);
    return __longlong_as_double(old);
}

template<typename Real>
__global__ void find_minmax_kernel(const Real* d_input, unsigned int n, Real* d_min, Real* d_max)
{
    extern __shared__ Real shared_mem[];

    unsigned int tid = threadIdx.x;
    unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;

    // Load shared memory with input values
    if (i < n)
    {
        shared_mem[tid] = d_input[i];
    }
    else
    {
        if (sizeof(Real) == sizeof(float))
        {
            shared_mem[tid] = FLT_MAX; // Initialize with max Real value
        }
        else
        {
            shared_mem[tid] = DBL_MAX; // Initialize with max Real value
        }
    }
    __syncthreads();

    // Reduce to find the minimum value
    for (unsigned int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (tid < stride && i + stride < n)
        {
            shared_mem[tid] = min<Real>(shared_mem[tid], shared_mem[tid + stride]);
        }
        __syncthreads();
    }

    // The first thread in the block writes the result to global memory
    if (tid == 0)
    {
        atomicMin<Real>(d_min, shared_mem[0]);
    }
    __syncthreads();

    // Reset shared memory for maximum reduction
    if (i < n)
    {
        shared_mem[tid] = d_input[i];
    }
    else
    {
        shared_mem[tid] = -FLT_MAX; // Initialize with min Real value
    }
    __syncthreads();

    // Reduce to find the maximum value
    for (unsigned int stride = blockDim.x / 2; stride > 0; stride >>= 1)
    {
        if (tid < stride && i + stride < n)
        {
            shared_mem[tid] = max<Real>(shared_mem[tid], shared_mem[tid + stride]);
        }
        __syncthreads();
    }

    // The first thread in the block writes the result to global memory
    if (tid == 0)
    {
        atomicMax<Real>(d_max, shared_mem[0]);
    }
}

template<typename Real>
void find_minmax(const Real* h_input, unsigned int n, Real* h_min, Real* h_max)
{
    Real* d_input, * d_min, * d_max;

    // Allocate memory on the GPU
    cudaMalloc((void**)&d_input, n * sizeof(Real));
    cudaMalloc((void**)&d_min, sizeof(Real));
    cudaMalloc((void**)&d_max, sizeof(Real));

    // Copy the input data to the GPU
    cudaMemcpy(d_input, h_input, n * sizeof(Real), cudaMemcpyHostToDevice);

    // Initialize min and max values on the GPU
    Real initial_min = sizeof(Real) == sizeof(float) ? FLT_MAX : DBL_MAX;
    Real initial_max = -(sizeof(Real) == sizeof(float) ? FLT_MAX : DBL_MAX);
    cudaMemcpy(d_min, &initial_min, sizeof(Real), cudaMemcpyHostToDevice);
    cudaMemcpy(d_max, &initial_max, sizeof(Real), cudaMemcpyHostToDevice);

    // Define the block and grid sizes
    unsigned int threadsPerBlock = 256;
    unsigned int blocksPerGrid = (n + threadsPerBlock - 1) / threadsPerBlock;

    // Launch the kernel
    find_minmax_kernel << <blocksPerGrid, threadsPerBlock, threadsPerBlock * sizeof(Real) >> > (d_input, n, d_min, d_max);

    // Copy the results back to the host
    cudaMemcpy(h_min, d_min, sizeof(Real), cudaMemcpyDeviceToHost);
    cudaMemcpy(h_max, d_max, sizeof(Real), cudaMemcpyDeviceToHost);

    // Free GPU memory
    cudaFree(d_input);
    cudaFree(d_min);
    cudaFree(d_max);
}