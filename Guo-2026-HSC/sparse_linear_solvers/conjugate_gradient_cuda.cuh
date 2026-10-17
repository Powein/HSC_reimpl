#pragma once
#include <utility>
#include "sparse_matrix_square/spmv_cuda.cuh"
#include "krylov_cuda.cuh"
#include "sparse_matrix_square/sparse_preconditioning/jacobi_cuda.cuh"
#include "sparse_matrix_square/sparse_preconditioning/block_jacobi_cuda.cuh"

template<typename Real, unsigned int block_size, class sparse_matrix>
inline std::pair<unsigned int, Real> conjugate_gradient_cuda(sparse_matrix& A, Real* b, Real* x, unsigned int dim, unsigned int max_iter = KRYLOV_MAX_ITER, Real tol = KRYLOV_EPSILON, bool isMemsetZero = true)
{
	unsigned int iter = 0;
	unsigned int vec_len = dim * block_size;
	//printf("vec_len  = %d\n", vec_len);
	Real rTr_prev, rTr, pTAp, alpha, beta;
	Real* r_prev;
	Real* r;
	Real* p;
	Real* Ap;
	cudaMalloc((void**)&r_prev, sizeof(Real) * vec_len);
	cudaMalloc((void**)&r, sizeof(Real) * vec_len);
	cudaMalloc((void**)&p, sizeof(Real) * vec_len);
	cudaMalloc((void**)&Ap, sizeof(Real) * vec_len);
	if (isMemsetZero == true) cudaMemset(x, 0, sizeof(Real) * vec_len);
	spmv_cuda<Real, block_size>(A, dim, x, Ap);
	axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, r_prev, 1, b, -1, Ap);
	rTr_prev = inner_product<Real>(vec_len, r_prev, r_prev);
	//printf("rTr = %f\n", rTr_prev);
	if (rTr_prev < tol)
	{
		cudaFree(r_prev);
		cudaFree(r);
		cudaFree(p);
		cudaFree(Ap);
		return { iter, rTr_prev };
	}
	cudaMemcpy(p, r_prev, sizeof(Real) * vec_len, cudaMemcpyDeviceToDevice);
	while (true)
	{
		if (max_iter > 0 && iter > max_iter) break;
		//std::cout << iter << std::endl;
		spmv_cuda<Real, block_size>(A, dim, p, Ap);
		pTAp = inner_product<Real>(vec_len, p, Ap);
		alpha = rTr_prev / pTAp;
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, x, 1, x, alpha, p);
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, r, 1, r_prev, -alpha, Ap);
		rTr = inner_product<Real>(vec_len, r, r);
		if (isnan(rTr))
		{
			printf("PCG error::rTr::NaN 1\n");
			exit(1);
		}
		//printf("%d rTr = %f\n", iter, rTr);
		if (rTr < tol) break;
		beta = rTr / rTr_prev;
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, p, 1, r, beta, p);
		rTr_prev = rTr;
		swap<Real*>(r_prev, r);
		++iter;
	}

	cudaFree(r_prev);
	cudaFree(r);
	cudaFree(p);
	cudaFree(Ap);
	return { iter, rTr };
}

//#pragma once
//#include <utility>
//#include <iostream>

template<typename Real, unsigned int block_size>
__global__ void dmv_accumulate_kernel(int dim, const Real* M_inv, const Real* r, Real* z, Real omega)
{
	int i = blockIdx.x * blockDim.x + threadIdx.x;
	if (i < dim)
	{
		int block_offset = i * block_size * block_size;
		int vec_offset = i * block_size;

		// 1. initialization
		Real res[block_size];
#pragma unroll
		for (int k = 0; k < block_size; ++k) res[k] = 0;

		// 2. compute D^-1 * r
#pragma unroll
		for (int row = 0; row < block_size; ++row)
		{
#pragma unroll
			for (int col = 0; col < block_size; ++col)
			{
				res[row] += M_inv[block_offset + row * block_size + col] * r[vec_offset + col];
			}
		}

		// 3. z = z + omega * D^-1 * r
#pragma unroll
		for (int row = 0; row < block_size; ++row)
		{
			z[vec_offset + row] += omega * res[row];
		}
	}
}

template<typename Real, unsigned int block_size>
void dmv_accumulate(int dim, const Real* M_inv, const Real* r, Real* z, Real omega)
{
	dmv_accumulate_kernel<Real, block_size> << <get_blocksPerGrid(dim), THREADS_PER_BLOCK >> > (dim, M_inv, r, z, omega);
}

template<typename Real, unsigned int block_size, class sparse_matrix>
inline std::pair<unsigned int, Real> PCG_cuda_neumann(sparse_matrix& A, Real* b, Real* x, unsigned int dim,
	unsigned int max_iter = KRYLOV_MAX_ITER, Real tol = KRYLOV_EPSILON, bool isMemsetZero = true,
	bool relative_residual = false, unsigned int neumann_iters = 1, Real omega = 0.6667)
{
	unsigned int iter = 0;
	unsigned int vec_len = dim * block_size;

	Real rTr_prev, rTr, rTz, rTz_prev, pTAp, alpha, beta;
	Real* r_prev, * r, * p, * Ap, * M_inv, * z_prev, * z, * t_k, * tmp_vec;

	cudaMalloc((void**)&r_prev, sizeof(Real) * vec_len);
	cudaMalloc((void**)&r, sizeof(Real) * vec_len);
	cudaMalloc((void**)&p, sizeof(Real) * vec_len);
	cudaMalloc((void**)&Ap, sizeof(Real) * vec_len);
	cudaMalloc((void**)&M_inv, sizeof(Real) * dim * block_size * block_size);
	cudaMalloc((void**)&z_prev, sizeof(Real) * vec_len);
	cudaMalloc((void**)&z, sizeof(Real) * vec_len);
	cudaMalloc((void**)&t_k, sizeof(Real) * vec_len);
	cudaMalloc((void**)&tmp_vec, sizeof(Real) * vec_len);

	auto free_workspace = [&]() {
		cudaFree(r_prev); cudaFree(r); cudaFree(p); cudaFree(Ap);
		cudaFree(M_inv); cudaFree(z_prev); cudaFree(z); cudaFree(t_k); cudaFree(tmp_vec);
	};

	compute_jacobi_preconditioner<Real, block_size>(A, dim, M_inv);

	if (isMemsetZero) cudaMemset(x, 0, sizeof(Real) * vec_len);

	spmv_cuda<Real, block_size>(A, dim, x, Ap);
	axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, r_prev, 1, b, -1, Ap);

	rTr_prev = inner_product<Real>(vec_len, r_prev, r_prev);
	if (rTr_prev < tol)
	{
		free_workspace();
		return { 0, rTr_prev };
	}
	if (relative_residual) tol *= rTr_prev;

	// t_0 = omega * D^-1 * r_prev
	cudaMemset(t_k, 0, sizeof(Real) * vec_len);
	dmv_accumulate<Real, block_size>(dim, M_inv, r_prev, t_k, omega);
	cudaMemcpy(z_prev, t_k, sizeof(Real) * vec_len, cudaMemcpyDeviceToDevice);

	for (unsigned int k = 1; k < neumann_iters; ++k)
	{
		spmv_cuda<Real, block_size>(A, dim, t_k, tmp_vec);
		// t_k = t_{k-1} - omega * D^-1 * (A * t_{k-1})
		dmv_accumulate<Real, block_size>(dim, M_inv, tmp_vec, t_k, -omega);
		// z_prev += t_k
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, z_prev, 1, z_prev, 1, t_k);
	}

	cudaMemcpy(p, z_prev, sizeof(Real) * vec_len, cudaMemcpyDeviceToDevice);
	rTz_prev = inner_product<Real>(vec_len, r_prev, z_prev);

	while (true)
	{
		if (max_iter > 0 && iter >= max_iter) break;

		spmv_cuda<Real, block_size>(A, dim, p, Ap);
		pTAp = inner_product<Real>(vec_len, p, Ap);
		alpha = rTz_prev / max<Real>(pTAp, EPSILON);

		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, x, 1, x, alpha, p);
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, r, 1, r_prev, -alpha, Ap);

		rTr = inner_product<Real>(vec_len, r, r);
		if (isnan(rTr)) { printf("PCG error::NaN\n"); exit(1); }
		if (rTr < tol) break;

		// --- Neumann Series ---
		// t_0 = omega * D^-1 * r
		cudaMemset(t_k, 0, sizeof(Real) * vec_len);
		dmv_accumulate<Real, block_size>(dim, M_inv, r, t_k, omega);
		cudaMemcpy(z, t_k, sizeof(Real) * vec_len, cudaMemcpyDeviceToDevice);

		for (unsigned int k = 1; k < neumann_iters; ++k)
		{
			spmv_cuda<Real, block_size>(A, dim, t_k, tmp_vec);
			dmv_accumulate<Real, block_size>(dim, M_inv, tmp_vec, t_k, -omega);
			axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, z, 1, z, 1, t_k);
		}

		rTz = inner_product<Real>(vec_len, r, z);
		beta = rTz / max<Real>(rTz_prev, EPSILON);
		axpby_kernel<Real> << <get_blocksPerGrid(vec_len), THREADS_PER_BLOCK >> > (vec_len, p, 1, z, beta, p);

		rTz_prev = rTz;
		swap<Real*>(r_prev, r);
		++iter;
	}

	free_workspace();
	return { iter, rTr };
}
