#pragma once
#include <cub/cub.cuh> 
#include <cub/util_allocator.cuh>
#include <cub/device/device_scan.cuh>
#include <cub/device/device_reduce.cuh>


template<class T>
__host__ __device__ __forceinline__
void inclusive_sum(unsigned int n, T* array_in, T* array_out)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    CubDebugExit(cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytes, array_in, array_out, n));
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    CubDebugExit(cub::DeviceScan::InclusiveSum(d_temp_storage, temp_storage_bytes, array_in, array_out, n));
    cudaFree(d_temp_storage);
}

template<class T>
__host__ __device__ __forceinline__
void exclusive_sum(unsigned int n, T* array_in, T* array_out)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    CubDebugExit(cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, array_in, array_out, n));
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    CubDebugExit(cub::DeviceScan::ExclusiveSum(d_temp_storage, temp_storage_bytes, array_in, array_out, n));
    cudaFree(d_temp_storage);
}

template<class T>
__host__ __device__ __forceinline__
void reduce_min(unsigned int n, T* array_in, T* result)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    CubDebugExit(cub::DeviceReduce::Min(d_temp_storage, temp_storage_bytes, array_in, result, n));
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    CubDebugExit(cub::DeviceReduce::Min(d_temp_storage, temp_storage_bytes, array_in, result, n));
    cudaFree(d_temp_storage);
}

template<class T>
__host__ __device__ __forceinline__
void reduce_max(unsigned int n, T* array_in, T* result)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    CubDebugExit(cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, array_in, result, n));
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    CubDebugExit(cub::DeviceReduce::Max(d_temp_storage, temp_storage_bytes, array_in, result, n));
    cudaFree(d_temp_storage);
}

template<class T>
__host__ __device__ __forceinline__
T reduce_sum(unsigned int n, T* array_in)
{
    T* result;
    cudaMalloc((void**)&result, sizeof(T));
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    CubDebugExit(cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, array_in, result, n));
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    CubDebugExit(cub::DeviceReduce::Sum(d_temp_storage, temp_storage_bytes, array_in, result, n));
    T result_host[1];
    cudaMemcpy(result_host, result, sizeof(T), cudaMemcpyDeviceToHost);
    cudaFree(d_temp_storage);
    cudaFree(result);
    return *result_host;
}

template<class T>
__host__ __device__ __forceinline__
void radix_sort(unsigned int n, T* array_in, T* array_out)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceRadixSort::SortKeys(d_temp_storage, temp_storage_bytes, array_in, array_out, n);
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    cub::DeviceRadixSort::SortKeys(d_temp_storage, temp_storage_bytes, array_in, array_out, n);
    cudaFree(d_temp_storage);
}

template<class T>
__host__ __device__ __forceinline__
void radix_sort_by_key(unsigned int n, T* key_in, T* key_out, T* value_in, T* value_out)
{
    void* d_temp_storage = NULL;
    size_t temp_storage_bytes = 0;
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, key_in, key_out, value_in, value_out, n);
    cudaMalloc(&d_temp_storage, temp_storage_bytes);
    cub::DeviceRadixSort::SortPairs(d_temp_storage, temp_storage_bytes, key_in, key_out, value_in, value_out, n);
    cudaFree(d_temp_storage);
}