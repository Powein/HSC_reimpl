#pragma once
#include <cuda_runtime.h>
#include "math/math_cuda.cuh"

template <typename Real>
__global__ void dense_matrix_matrix_multiplication_kernel(unsigned int A, unsigned int B, unsigned int C, Real* matL, Real* matR, Real* result)
{
    unsigned int N = A * C;
    unsigned int e = blockDim.x * blockIdx.x + threadIdx.x;
    if (e >= N) return;
    unsigned int r = e / C;
    unsigned int c = e % C;
    Real matmul_acc = 0.;
    for (unsigned int i = 0; i < B; ++i)
    {
        matmul_acc += matL[B * r + i] * matR[C * i + c];
    }
    result[C * r + c] = matmul_acc;
}

template <typename Real, unsigned int SHMEM_SIZE>
__global__ void dense_matrix_matrix_multiplication_tiled_kernel(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    // Compute each thread's global row and column index
    unsigned int row = blockIdx.y * blockDim.y + threadIdx.y;
    unsigned int col = blockIdx.x * blockDim.x + threadIdx.x;

    // Statically allocated shared memory
    __shared__ Real s_a[SHMEM_SIZE];
    __shared__ Real s_b[SHMEM_SIZE];

    // Accumulate in temporary variable
    Real matmul_acc = 0.;

    // Loop over the tiles
    for (unsigned int i = 0; i < K; i += blockDim.x) {
        // Load elements for this tile
        if (row < M && i + threadIdx.x < K)
        {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = a[row * K + i + threadIdx.x];
        }
        else
        {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        if (col < N && i + threadIdx.y < K)
        {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = b[(i + threadIdx.y) * N + col];
        }
        else
        {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        // Wait for both tiles to be loaded before computation
        __syncthreads();

        // Do matrix multiplication on the small matrix
        for (unsigned int j = 0; j < blockDim.x; ++j)
        {
            matmul_acc += s_a[threadIdx.y * blockDim.x + j] * s_b[j * blockDim.x + threadIdx.x];
        }

        // Wait for all threads to finish using current tiles before loading new ones
        __syncthreads();
    }

    // Write back results only if within bounds
    if (row < M && col < N)
    {
        c[row * N + col] = matmul_acc;
    }
}

template <typename Real, unsigned int m>
void dense_matrix_matrix_multiplication_tiled(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    dim3 blockDim(m, m);
    dim3 gridDim((N + blockDim.x - 1) / blockDim.x, (M + blockDim.y - 1) / blockDim.y);
    dense_matrix_matrix_multiplication_tiled_kernel<Real, m * m> << <gridDim, blockDim >> > (M, K, N, a, b, c);
}

template <typename Real>
__global__ void dense_matrixT_matrix_multiplication_kernel(unsigned int A, unsigned int B, unsigned int C, Real* matL, Real* matR, Real* result)
{
    unsigned int N = A * C;
    unsigned int e = blockDim.x * blockIdx.x + threadIdx.x;
    if (e >= N) return;
    unsigned int r = e / C;
    unsigned int c = e % C;
    Real matmul_acc = 0.;
    for (unsigned int i = 0; i < B; ++i)
    {
        matmul_acc += matL[A * i + r] * matR[C * i + c];
    }
    result[C * r + c] = matmul_acc;
}

template <typename Real, unsigned int SHMEM_SIZE>
__global__ void dense_matrixT_matrix_multiplication_tiled_kernel(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    // Compute each thread's global row and column index
    unsigned int row = blockIdx.y * blockDim.y + threadIdx.y; // Row index for output C
    unsigned int col = blockIdx.x * blockDim.x + threadIdx.x; // Column index for output C

    // Statically allocated shared memory
    __shared__ Real s_a[SHMEM_SIZE]; // Shared memory for transposed matrix A
    __shared__ Real s_b[SHMEM_SIZE]; // Shared memory for matrix B

    // Accumulate in temporary variable
    Real matmul_acc = 0.;

    // Loop over the tiles
    for (unsigned int i = 0; i < K; i += blockDim.x) {
        // Load elements for this tile of A^T (KxM)
        if (i + threadIdx.x < K && row < M) {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = a[(i + threadIdx.x) * M + row]; // Accessing transposed A
        }
        else {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        // Load elements for this tile of B (KxN)
        if (i + threadIdx.y < K && col < N) {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = b[(i + threadIdx.y) * N + col];
        }
        else {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        // Wait for both tiles to be loaded before computation
        __syncthreads();

        // Do matrix multiplication on the small matrix
        for (unsigned int j = 0; j < blockDim.x; ++j) {
            matmul_acc += s_a[threadIdx.y * blockDim.x + j] * s_b[j * blockDim.x + threadIdx.x];
        }

        // Wait for all threads to finish using current tiles before loading new ones
        __syncthreads();
    }

    // Write back results only if within bounds
    if (row < M && col < N) {
        c[row * N + col] = matmul_acc;
    }
}

template <typename Real, unsigned int m>
void dense_matrixT_matrix_multiplication_tiled(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    dim3 blockDim(m, m);
    dim3 gridDim((N + blockDim.x - 1) / blockDim.x, (M + blockDim.y - 1) / blockDim.y);
    dense_matrixT_matrix_multiplication_tiled_kernel<Real, m * m> << <gridDim, blockDim >> > (M, K, N, a, b, c);
}

template <typename Real>
__global__ void dense_matrix_matrixT_multiplication_kernel(unsigned int A, unsigned int B, unsigned int C, Real* matL, Real* matR, Real* result)
{
    unsigned int N = A * C;
    unsigned int e = blockDim.x * blockIdx.x + threadIdx.x;
    if (e >= N) return;
    unsigned int r = e / C;
    unsigned int c = e % C;
    Real matmul_acc = 0.;
    for (unsigned int i = 0; i < B; ++i)
    {
        matmul_acc += matL[B * r + i] * matR[B * c + i];
    }
    result[C * r + c] = matmul_acc;
}

template <typename Real, unsigned int SHMEM_SIZE>
__global__ void dense_matrix_matrixT_multiplication_tiled_kernel(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    // Compute each thread's global row and column index
    unsigned int row = blockIdx.y * blockDim.y + threadIdx.y; // Row index for output C
    unsigned int col = blockIdx.x * blockDim.x + threadIdx.x; // Column index for output C

    // Statically allocated shared memory
    __shared__ Real s_a[SHMEM_SIZE]; // Shared memory for matrix A
    __shared__ Real s_b[SHMEM_SIZE]; // Shared memory for transposed matrix B

    // Accumulate in temporary variable
    Real matmul_acc = 0.;

    // Loop over the tiles
    for (unsigned int i = 0; i < K; i += blockDim.x) {
        // Load elements for this tile of A (MxK)
        if (row < M && i + threadIdx.x < K) {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = a[row * K + (i + threadIdx.x)];
        }
        else {
            s_a[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        // Load elements for this tile of B^T (KxN)
        if (i + threadIdx.y < K && col < N) {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = b[col * K + (i + threadIdx.y)];
        }
        else {
            s_b[threadIdx.y * blockDim.x + threadIdx.x] = 0; // Padding for out of bounds
        }

        // Wait for both tiles to be loaded before computation
        __syncthreads();

        // Do matrix multiplication on the small matrix
        for (unsigned int j = 0; j < blockDim.x; ++j) {
            matmul_acc += s_a[threadIdx.y * blockDim.x + j] * s_b[j * blockDim.x + threadIdx.x];
        }

        // Wait for all threads to finish using current tiles before loading new ones
        __syncthreads();
    }

    // Write back results only if within bounds
    if (row < M && col < N) {
        c[row * N + col] = matmul_acc;
    }
}

template <typename Real, unsigned int m>
void dense_matrix_matrixT_multiplication_tiled(unsigned int M, unsigned int K, unsigned int N, Real* a, Real* b, Real* c)
{
    dim3 blockDim(m, m);
    dim3 gridDim((N + blockDim.x - 1) / blockDim.x, (M + blockDim.y - 1) / blockDim.y);
    dense_matrix_matrixT_multiplication_tiled_kernel<Real, m* m> << <gridDim, blockDim >> > (M, K, N, a, b, c);
}

template <typename Real>
__global__ void dense_matrix_transpose_kernel(unsigned int R, unsigned int C, Real* mat_in, Real* mat_out)
{
    unsigned int N = R * C;
    unsigned int e = blockDim.x * blockIdx.x + threadIdx.x;
    if (e >= N) return;
    unsigned int r = e / C;
    unsigned int c = e % C;
    mat_out[C * r + c] = mat_in[R * c + r];
}