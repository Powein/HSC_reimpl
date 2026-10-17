#pragma once
#include <cuda_runtime.h>
#include "euler_rotation_cuda.cuh"

// return axis and theta
template <typename Real>
__host__ __device__ __forceinline__
void euler_to_rodrigues(Real alpha, Real beta, Real gamma, Real* axis, Real& theta, Real* R)
{
    euler_to_rotation<Real>(alpha, beta, gamma, R);
    theta = acos((R[0] + R[4] + R[8] - 1.) * 0.5);
    Real coeff = 0.5 / sin(theta);
    axis[0] = (R[6] - R[2]) * coeff;
    axis[1] = (R[5] - R[7]) * coeff;
    axis[2] = (R[1] - R[3]) * coeff;
}

template <typename Real>
__host__ __device__ __forceinline__
void euler_to_rodrigues(Real* euler, Real* axis, Real& theta, Real* R)
{
    euler_to_rodrigues<Real>(euler[0], euler[1], euler[2], axis, theta, R);
}

template <typename Real>
__host__ __device__ __forceinline__
void rodrigues(Real* axis, Real& theta, Real* R)
{
    Real k0 = axis[0];
    Real k1 = axis[1];
    Real k2 = axis[2];
    Real t2 = cos(theta);
    Real t3 = sin(theta);
    Real t4 = k0 * k0;
    Real t5 = k1 * k1;
    Real t6 = k2 * k2;
    Real t7 = k0 * t3;
    Real t8 = k1 * t3;
    Real t9 = k2 * t3;
    Real t10 = t2 - 1.0;
    Real t11 = k0 * k1 * t10;
    Real t12 = k0 * k2 * t10;
    Real t13 = k1 * k2 * t10;
    Real t14 = t4 * t10;
    Real t15 = t5 * t10;
    Real t16 = t6 * t10;
    Real t17 = -t11;
    Real t18 = -t12;
    Real t19 = -t13;
    R[0] = t15 + t16 + 1.0;
    R[1] = -t9 + t17;
    R[2] = t8 + t18;
    R[3] = t9 + t17;
    R[4] = t14 + t16 + 1.0;
    R[5] = -t7 + t19;
    R[6] = -t8 + t18;
    R[7] = t7 + t19;
    R[8] = t14 + t15 + 1.0;
}