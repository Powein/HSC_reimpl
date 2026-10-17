# Heterogeneous Subspace Corrections for GPU Deformable Multibody Dynamics (SIGGRAPH 2026)

This repository contains the accompanying CUDA/C++ code for HSC-Multibody (SIGGRAPH 2026). It provides GPU-oriented numerical kernels for block-sparse matrix assembly and Krylov linear solvers. The implementation is organized as a collection of CUDA header files and is intended to be integrated into a larger simulation or optimization codebase.

The code uses template parameters extensively to specify scalar types and block dimensions. It is particularly suitable for applications involving finite element methods, affine body dynamics, coupled FEM/affine systems, geometric constraints, and GPU-based sparse linear algebra.

At present, the repository does not include a standalone build system, executable demo, or benchmark driver. It should therefore be treated as a header-only CUDA utility library. Users should include the relevant `.cuh` files from their own CUDA/C++ project and add the repository root to the compiler include path.

This repository provides a reference reimplementation to support the reproducibility of our paper and clarify its underlying algorithms (affine assembly, rotational dynamics, Neumann-series preconditioner, and PCG). As a research-focused reimplementation, this code is not intended or optimized for production environments. It intentionally omits standard engineering features such as error handling, input validation, platform-specific build setups, and deep performance tuning.

Dewen Guo (PhD, Peking University) guodw.sh@gmail.com

## Repository Structure

```text
affine_projection/        Affine projection of gradients and Hessians
math/                     Basic math, matrix operations, rotations, SVD, reductions, and small linear algebra routines
rotational/               Gradients and Hessians for rotational terms
sparse_matrix_square/     Block-sparse matrix data structures, assembly, SpMV, and preconditioners
sparse_linear_solvers/    CUDA Krylov solvers, including CG and PCG variants
utils/                    Debug output, CSV/OBJ export, CUDA launch configuration, and initialization utilities
```

## Main Components

### Affine Projection

`affine_projection/affine_projection_cuda.cuh` provides routines for projecting position-space derivatives to affine degrees of freedom. The main functions include:

- `affine_projection`: constructs the projection matrix from a 3D rest position to affine degrees of freedom.
- `affine_projection_gradient3x1`: maps a 3D gradient to a 12-dimensional affine gradient.
- `affine_projection_Hessian3x3`: maps a 3x3 Hessian to a 12x12 affine Hessian.
- `affine_projection_gradientVF/EE/VE/VV` and the corresponding Hessian routines: project derivatives for vertex-face, edge-edge, vertex-edge, and vertex-vertex geometric interactions.

These routines are implemented as templated `__host__ __device__ __forceinline__` functions and can be called either from host code or inside CUDA kernels.

### Math Utilities

The `math/` directory contains small-scale numerical utilities used by the rest of the codebase:

- `math_cuda.cuh`: vector operations, matrix operations, dot products, cross products, matrix-vector products, and transposed products.
- `svd3_cuda.cuh`: singular value decomposition and polar decomposition for 3x3 matrices.
- `euler_rotation_cuda.cuh` and `euler_rodrigues_cuda.cuh`: conversions between Euler angles, rotation matrices, and Rodrigues representations.
- `cholesky_cuda.cuh`, `gaussian_elimination_cuda.cuh`, and `gauss_jordan_cuda.cuh`: small dense linear system solvers and matrix inversion routines.
- `reduce_cuda.cuh` and `kernel_computations/`: reductions, sorting, inner products, min/max queries, and dense matrix multiplication kernels.

### Block-Sparse Matrices

The `sparse_matrix_square/` directory implements several block-sparse matrix representations and matrix-vector multiplication kernels:

- `sparse_matrix_cuda<Real, block_size>`: a general square block-sparse matrix represented by row offsets and column indices.
- `sparse_matrix_symmetric_cuda<Real, block_size>`: a symmetric block-sparse matrix with separate storage for diagonal and lower-triangular blocks.
- `sparse_blocks_cuda<Real, block_size>`: a coordinate-based block storage format, useful during matrix assembly.
- `sparse_blocks_nonsquare_cuda<Real, block_size_r, block_size_c>`: a non-square block matrix representation.
- `spmv_cuda.cuh` and `spmv_offdiag_cuda.cuh`: CUDA SpMV kernels for the above sparse matrix types.
- `sparse_assembly_cuda.cuh`: aggregate sparse matrix structures for unified scaling and multiplication.
- `sparse_assembly_fem_abd_cuda.cuh`: sparse matrix assembly and multiplication for coupled FEM and affine body dynamics systems.
- `customized_sparse_matrix/sparse_KKT_cuda.cuh`: specialized KKT matrix structure and related solver utilities.

Preconditioning routines are provided in `sparse_matrix_square/sparse_preconditioning/`:

- `jacobi_cuda.cuh`: block-Jacobi preconditioners, diagonal block extraction, and diagonal block inversion.
- `block_jacobi_cuda.cuh`: block-diagonal inversion and block-diagonal matrix-vector products.
- `diagonal_accumulation_cuda.cuh`: accumulation of diagonal preconditioner blocks with configurable block sizes.

### Sparse Linear Solvers

The `sparse_linear_solvers/` directory provides CUDA implementations of Krylov solvers:

- `conjugate_gradient_cuda`: standard conjugate gradient solver.
- `PCG_cuda_neumann`: preconditioned conjugate gradient solver using a Jacobi/Neumann-series approximation.
- `PCG_fem_abd_coupling_cuda`: preconditioned conjugate gradient solver for coupled FEM and affine body dynamics systems with 3D FEM degrees of freedom and 12D affine degrees of freedom.
- `krylov_cuda.cuh`: default solver parameters, including `KRYLOV_EPSILON` and `KRYLOV_MAX_ITER`.

The solvers generally assume that the matrix, right-hand side, and solution vectors are already allocated on the device. They return a pair containing the number of iterations and the final residual.

### Rotational Terms

`rotational/rotational_cuda.cuh` contains derivative computations for rotational terms, including:

- `rotational_gradient`
- `rotational_Hessian`

These routines are intended for local energy or constraint terms that depend on rest positions, current degrees of freedom, and rotational quantities.

### Utility Routines

The `utils/` directory contains auxiliary functionality:

- `threads_per_block.cuh`: defines `THREADS_PER_BLOCK` and provides `get_blocksPerGrid`.
- `memset_any_cuda.cuh`: initializes device arrays with arbitrary values and sequential integer indices.
- `print_debug_cuda.cuh`: copies device data to the host for inspection and exports debugging geometry in OBJ/TOBJ formats.
- `export_csv.hpp`: exports host-side arrays to CSV files.

## Dependencies

The code depends on:

- CUDA Runtime: `cuda_runtime.h` and `cuda.h`
- Thrust: used for selected reductions, sorting, and device-vector operations
- Eigen: `Eigen/Core` is included by `math/math_cuda.cuh`
- C++ standard library components such as `vector`, `memory`, `utility`, and `iostream`

Compilation should be performed with a CUDA-capable C++ compiler such as `nvcc`. The repository root should be added to the include path.

## Basic Usage

Include the required headers from a CUDA/C++ translation unit, for example:

```cpp
#include "sparse_matrix_square/sparse_matrix_cuda.cuh"
#include "sparse_matrix_square/spmv_cuda.cuh"
#include "sparse_linear_solvers/conjugate_gradient_cuda.cuh"
```

A typical workflow is:

1. Construct the sparse topology on the host, for example using `rows_to_cols` and `offsets`.
2. Instantiate the corresponding GPU sparse matrix type, such as `sparse_matrix_cuda<Real, block_size>`.
3. Fill the device-side nonzero blocks `dev_nonzeros` and allocate the right-hand side `b` and solution vector `x`.
4. Call `spmv_cuda`, `conjugate_gradient_cuda`, or `PCG_cuda_neumann`.
5. Copy the result back to the host when needed for validation, visualization, or output.

## Intended Use

This code is intended for research use in GPU-based simulation and numerical optimization. Typical use cases include:

- block-sparse linear system assembly and solution on the GPU;
- coupled finite element and affine body dynamics systems;
- projection of geometric gradients and Hessians to affine degrees of freedom;
- small dense matrix decompositions and rotation conversions inside CUDA kernels.

## License

This project is released under the BSD 3-Clause License.

This repository includes a modified GPU implementation of the 3x3 SVD routine in `math/svd3_cuda.cuh`, originally attributed in the source header to Eric Jang and based on McAdams et al., "Computing the Singular Value Decomposition of 3x3 matrices with minimal branching and elementary operations."

## Disclaimer

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.