#pragma once
#ifndef THREADS_PER_BLOCK
#define THREADS_PER_BLOCK 256
#endif

#define CUDA_CHECK(call) \
    { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error: " << cudaGetErrorString(err) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            exit(EXIT_FAILURE); \
        } \
    }

inline unsigned int get_blocksPerGrid(unsigned int n)
{
    n = n == 0u ? 1u : n;
    return (n + THREADS_PER_BLOCK - 1) / THREADS_PER_BLOCK;
}