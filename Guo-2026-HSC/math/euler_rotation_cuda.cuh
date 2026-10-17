#pragma once
#include "math_cuda.cuh"

template <typename Real>
__host__ __device__ __forceinline__
void euler_to_rotation(Real alpha, Real beta, Real gamma, Real* R)
{
    alpha = radians<Real>(alpha);
    beta = radians<Real>(beta);
    gamma = radians<Real>(gamma);
    memset(R, 0, sizeof(Real) * 9);
    Real t2 = cos(alpha);
    Real t3 = cos(beta);
    Real t4 = cos(gamma);
    Real t5 = sin(alpha);
    Real t6 = sin(beta);
    Real t7 = sin(gamma);
    R[0] = t3 * t4;
    R[1] = -t2 * t7 + t4 * t5 * t6;
    R[2] = t5 * t7 + t2 * t4 * t6;
    R[3] = t3 * t7;
    R[4] = t2 * t4 + t5 * t6 * t7;
    R[5] = -t4 * t5 + t2 * t6 * t7;
    R[6] = -t6;
    R[7] = t3 * t5;
    R[8] = t2 * t3;
}

template <typename Real>
__host__ __device__ __forceinline__
void euler_to_rotation(Real* euler, Real* R)
{
    euler_to_rotation<Real>(euler[0], euler[1], euler[2], R);
}

//Ken Shoemake, Animating Rotation with Quaternion Curves Proc. SIGGRAPH 1985, pp. 245- 254.
//Computing Euler angles from a rotation matrix (Gregory G. Slabaugh)
//2 possible results
template <typename Real>
__host__ __device__ __forceinline__
void rotation_to_euler(Real* R, Real* euler0, Real* euler1)
{
    if (abs<Real>(R[6]) != 1.)
    {
        euler0[1] = -asin(R[6]);
        euler1[1] = PI - euler0[1];
        euler0[0] = atan2(R[7] / euler0[1], R[8] / euler0[1]);
        euler1[0] = atan2(R[7] / euler1[1], R[8] / euler1[1]);
        euler0[2] = atan2(R[3] / euler0[1], R[0] / euler0[1]);
        euler1[2] = atan2(R[3] / euler1[1], R[0] / euler1[1]);
    }
    else
    {
        euler0[2] = 0.;
        if (R[6] == -1.)
        {
            euler0[1] = 0.5 * PI;
            euler0[0] = euler0[2] + atan2(R[1], R[2]);
        }
        else
        {
            euler0[1] = -0.5 * PI;
            euler0[0] = -euler0[2] + atan2(-R[1], -R[2]);
        }
        euler1[0] = euler0[0];
        euler1[1] = euler0[1];
        euler1[2] = euler0[2];
    }
}
