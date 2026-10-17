#pragma once
#include <fstream>
#include <sstream>
#include <cuda_runtime.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <thrust/device_vector.h>
#include "utils/export_csv.hpp"

template<class T>
inline void print_debug(const char* log_file, unsigned int n, unsigned int m, T* device_array)
{
    std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	T* host_array = new T[n];
	cudaMemcpy(host_array, device_array, sizeof(T) * n, cudaMemcpyDeviceToHost);
	//outfile << "start\n";
	for (unsigned int i = 0; i < n / m; ++i)
	{
        for(unsigned int j = 0; j < m; ++j)
        {
			outfile << std::setprecision(12) << host_array[m * i + j] << " ";
        }
        outfile << std::endl;
	}
	//outfile << "end\n";
	outfile.close();
	delete[] host_array;
}

template<class T>
inline void print_debug(const char* log_file, unsigned int n, unsigned int m, std::vector<T> host_array)
{
	std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	//outfile << "start\n";
	for (unsigned int i = 0; i < n / m; ++i)
	{
		for (unsigned int j = 0; j < m; ++j)
		{
			outfile << std::setprecision(12) << host_array[m * i + j] << " ";
		}
		outfile << std::endl;
	}
	//outfile << "end\n";
	outfile.close();
}

template<class T>
inline void print_debug(unsigned int n, T* device_array)
{
	T* host_array = new T[n];
	cudaMemcpy(host_array, device_array, sizeof(T) * n, cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < n; ++i)
	{
		std::cout << i << " " << std::setprecision(20) << host_array[i] << std::endl;
		//printf("%d %f\n", i, host_array[i]);
	}
	delete[] host_array;
}

template<class T>
inline void print_debug(unsigned int n, unsigned int m, T* device_array)
{
	T* host_array = new T[n * m];
	cudaMemcpy(host_array, device_array, sizeof(T) * n * m, cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < n; ++i)
	{
		//std::cout << i << ":";
		for (unsigned int j = 0; j < m; ++j)
		{
			std::cout << " " << host_array[m * i + j];
		}
		std::cout << std::endl;
	}
	delete[] host_array;
}

template<class T>
inline T visit_device(T*& device_array, unsigned int n)
{
	T host_array[1];
	//cudaMemcpy(host_array, device_array + n, sizeof(T), cudaMemcpyDeviceToHost);
	CUDA_CHECK(cudaMemcpy(host_array, device_array + n, sizeof(T), cudaMemcpyDeviceToHost));
	T value = host_array[0];
	return value;

	//thrust::device_vector<T> d_vec(device_array, device_array + n + 1);
	//return d_vec[n];
}

template<class Real>
inline void export_obj(unsigned int nVerts, Real* verts, unsigned int nTris, unsigned int* tris, const char* log_file, unsigned int type)
{
	std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	Real* host_verts = new Real[nVerts * 3];
	cudaMemcpy(host_verts, verts, sizeof(Real) * nVerts * 3, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	unsigned int* host_tris = new unsigned int[nTris * 3];
	cudaMemcpy(host_tris, tris, sizeof(unsigned int) * nTris * 3, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < nVerts; ++i)
	{
		outfile << "v " << std::fixed << std::setprecision(10) << host_verts[3 * i + 0]
			<< " " << std::fixed << std::setprecision(10) << host_verts[3 * i + 1]
			<< " " << std::fixed << std::setprecision(10) << host_verts[3 * i + 2] << std::endl;
	}
	for (unsigned int i = 0; i < nTris; ++i)
	{
		outfile << "f " << host_tris[3 * i + 0] + 1 << " " << host_tris[3 * i + 1] + 1 << " " << host_tris[3 * i + 2] + 1 << std::endl;
	}
	outfile.close();
	delete[] host_verts;
	delete[] host_tris;
}

template<class Real>
inline void export_obj_segments(unsigned int nVerts, Real* verts, unsigned int nSegs, unsigned int* segs, const char* log_file, unsigned int type)
{
	std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	Real* host_verts = new Real[nVerts * 3];
	cudaMemcpy(host_verts, verts, sizeof(Real) * nVerts * 3, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	unsigned int* host_segs = new unsigned int[nSegs * 2];
	cudaMemcpy(host_segs, segs, sizeof(unsigned int) * nSegs * 2, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);

	for (unsigned int i = 0; i < nVerts; ++i)
	{
		outfile << "v " << std::fixed << std::setprecision(20) << host_verts[3 * i + 0]
			<< " " << std::fixed << std::setprecision(20) << host_verts[3 * i + 1]
			<< " " << std::fixed << std::setprecision(20) << host_verts[3 * i + 2] << std::endl;
	}
	for (unsigned int i = 0; i < nSegs; ++i)
	{
		outfile << "l " << host_segs[2 * i + 0] + 1 << " " << host_segs[2 * i + 1] + 1 << " " << std::endl;
	}
	outfile.close();

	delete[] host_verts;
	delete[] host_segs;
}

template<class Real>
inline void export_tobj(unsigned int nVerts, Real* verts, unsigned int nTris, unsigned int* tris, unsigned int nTets, unsigned int* tets, const char* log_file, unsigned int type)
{
	std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	Real* host_verts = new Real[nVerts * 3];
	cudaMemcpy(host_verts, verts, sizeof(Real) * nVerts * 3, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	unsigned int* host_tris = new unsigned int[nTris * 3];
	cudaMemcpy(host_tris, tris, sizeof(unsigned int) * nTris * 3, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	unsigned int* host_tets = new unsigned int[nTets * 4];
	cudaMemcpy(host_tets, tets, sizeof(unsigned int) * nTets * 4, type == 0u ? cudaMemcpyHostToHost : cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < nVerts; ++i)
	{
		outfile << "v " << std::fixed << std::setprecision(10) << host_verts[3 * i + 0]
			<< " " << std::fixed << std::setprecision(10) << host_verts[3 * i + 1]
			<< " " << std::fixed << std::setprecision(10) << host_verts[3 * i + 2] << std::endl;
	}
	for (unsigned int i = 0; i < nTris; ++i)
	{
		outfile << "f " << host_tris[3 * i + 0] + 1 << " " << host_tris[3 * i + 1] + 1 << " " << host_tris[3 * i + 2] + 1 << std::endl;
	}
	for (unsigned int i = 0; i < nTets; ++i)
	{
		outfile << "t " << host_tets[4 * i + 0] + 1 << " " << host_tets[4 * i + 1] + 1 << " " << host_tets[4 * i + 2] + 1 << " " << host_tets[4 * i + 3] + 1 << std::endl;
	}

	outfile.close();
	delete[] host_verts;
	delete[] host_tris;
	delete[] host_tets;
}

inline void export_offset_info(std::vector<unsigned int>& offsets, const char* file_path)
{
	std::ofstream outfile;
	outfile.open(file_path, std::ios::out);
	outfile << offsets.size() << std::endl;
	for (unsigned int i = 0; i < offsets.size(); ++i)
	{
		outfile << offsets[i] << std::endl;
	}
	outfile.close();
}

template <typename T>
inline void print_debug_offseted_array(unsigned int N, unsigned int* offsets, T* array)
{
	unsigned int arrayLen = visit_device<unsigned int>(offsets, N);
	unsigned int* host_offsets = new unsigned int[N + 1];
	T* host_array = new T[arrayLen];
	cudaMemcpy(host_offsets, offsets, sizeof(unsigned int) * (N + 1), cudaMemcpyDeviceToHost);
	cudaMemcpy(host_array, array, sizeof(T) * arrayLen, cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < N; ++i)
	{
		if (host_offsets[i + 1] - host_offsets[i] == 0u) continue;
		printf("%d (%d):", i, host_offsets[i + 1] - host_offsets[i]);
		for (unsigned int j = host_offsets[i]; j < host_offsets[i + 1]; ++j)
		{
			if constexpr (std::is_same<T, unsigned int>::value)
			{
				printf(" %u", host_array[j]);
			}
			else if constexpr (std::is_same<T, int>::value)
			{
				printf(" %d", host_array[j]);
			}
			else if constexpr (std::is_same<T, double>::value || std::is_same<T, float>::value)
			{
				printf(" %f", host_array[j]);
			}
		}
		printf("\n");
	}
	delete[] host_offsets;
	delete[] host_array;
}

template <typename T>
inline void print_debug_id_array(unsigned int N, unsigned int* ids, T* array)
{
	unsigned int* host_ids = new unsigned int[N];
	T* host_array = new T[N];
	cudaMemcpy(host_ids, ids, sizeof(unsigned int) * N, cudaMemcpyDeviceToHost);
	cudaMemcpy(host_array, array, sizeof(T) * N, cudaMemcpyDeviceToHost);
	for (unsigned int i = 0; i < N; ++i)
	{
		if constexpr (std::is_same<T, unsigned int>::value)
		{
			printf("%d: %u\n", host_ids[i], host_array[i]);
		}
		else if constexpr (std::is_same<T, int>::value)
		{
			printf("%d: %d\n", host_ids[i], host_array[i]);
		}
		else if constexpr (std::is_same<T, double>::value || std::is_same<T, float>::value)
		{
			printf("%d: %f\n", host_ids[i], host_array[i]);
		}
	}
	delete[] host_ids;
	delete[] host_array;
}