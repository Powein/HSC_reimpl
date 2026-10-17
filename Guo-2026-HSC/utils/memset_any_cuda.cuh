#pragma once
#include <cuda_runtime.h>
#include "utils/threads_per_block.cuh"

template<typename Real>
__global__ void memset_any_kernel(unsigned int n, Real* data, Real value)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
	if (i >= n) return;
	data[i] = value;
}

template<typename Integer>
__global__ void init_seq_kernel(unsigned int n, Integer* data)
{
	unsigned int i = blockDim.x * blockIdx.x + threadIdx.x;
	if (i >= n) return;
	data[i] = i;
}

template<typename Real>
inline void memset_any(Real* data, Real value, unsigned int n)
{
	unsigned int blocksPerGrid = get_blocksPerGrid(n);
	memset_any_kernel<Real> << <blocksPerGrid, THREADS_PER_BLOCK >> > (n, data, value);
}

template<typename Integer>
inline void init_seq(Integer* data, unsigned int n)
{
	init_seq_kernel<Integer> << < get_blocksPerGrid(n), THREADS_PER_BLOCK >> > (n, data);
}