
__device__ void* memset(void* address, int value, unsigned long long size) {
    unsigned char* bytes = (unsigned char*)address;
    for (unsigned long long i=0; i<size; ++i) bytes[i] = (unsigned char)value;
    return address;
}

// Extracted without edits: math/math_cuda.cuh : vecSubs
template <class T>
__host__ __device__ __forceinline__ void vecSubs(T* X, const T* A, const T* B, int n)
{
    for (int i = 0; i < n; ++i)
        X[i] = A[i] - B[i];
}

// Extracted without edits: math/math_cuda.cuh : dot3
template <class T>
__host__ __device__ __forceinline__ T dot3(const T* vec1, const T* vec2)
{
    T result = 0.0;
    for (int i = 0; i < 3; ++i)
        result += vec1[i] * vec2[i];
    return result;
}

// Extracted without edits: math/math_cuda.cuh : affine_transform
template<typename Real>
__host__ __device__ __forceinline__ void affine_transform(Real* DOF, Real* rest_x, Real* x)
{
    x[0] = DOF[0] * rest_x[0] + DOF[1] * rest_x[1] + DOF[2] * rest_x[2] + DOF[9];
    x[1] = DOF[3] * rest_x[0] + DOF[4] * rest_x[1] + DOF[5] * rest_x[2] + DOF[10];
    x[2] = DOF[6] * rest_x[0] + DOF[7] * rest_x[1] + DOF[8] * rest_x[2] + DOF[11];
}

// Extracted without edits: affine_projection/affine_projection_cuda.cuh : affine_projection
template<typename Real>
__host__ __device__ __forceinline__ void affine_projection(Real* rest_x, Real* P)
{
    memset(P, 0, sizeof(Real) * 36);
    Real x0 = rest_x[0]; Real x1 = rest_x[1]; Real x2 = rest_x[2];
    P[0] = x0; P[1] = x1; P[2] = x2; P[9] = 1.0;
    P[15] = x0; P[16] = x1; P[17] = x2; P[22] = 1.0;
    P[30] = x0; P[31] = x1; P[32] = x2; P[35] = 1.0;
}

// Extracted without edits: affine_projection/affine_projection_cuda.cuh : affine_projection_gradient3x1
template<typename Real> // P = (dxdq)T; (dxdq)T * (g)_{3x1}
__host__ __device__ __forceinline__ void affine_projection_gradient3x1(Real* rest_x, Real* g, Real* g_affine)
{
    Real x0 = rest_x[0]; Real x1 = rest_x[1]; Real x2 = rest_x[2];
    g_affine[0] = g[0] * x0;
    g_affine[1] = g[0] * x1;
    g_affine[2] = g[0] * x2;
    g_affine[3] = g[1] * x0;
    g_affine[4] = g[1] * x1;
    g_affine[5] = g[1] * x2;
    g_affine[6] = g[2] * x0;
    g_affine[7] = g[2] * x1;
    g_affine[8] = g[2] * x2;
    g_affine[9] = g[0];
    g_affine[10] = g[1];
    g_affine[11] = g[2];
}

// Extracted without edits: affine_projection/affine_projection_cuda.cuh : affine_projection_Hessian3x3
template<typename Real> // (dxdq)T * (H)_{3x3} * dxdq
__host__ __device__ __forceinline__ void affine_projection_Hessian3x3(Real* rest_x, Real* H, Real* H_affine)
{
    Real x0 = rest_x[0]; Real x1 = rest_x[1]; Real x2 = rest_x[2];
    Real t2 = H[0] * x0;
    Real t3 = H[0] * x1;
    Real t4 = H[1] * x0;
    Real t5 = H[0] * x2;
    Real t6 = H[1] * x1;
    Real t7 = H[2] * x0;
    Real t8 = H[1] * x2;
    Real t9 = H[2] * x1;
    Real t10 = H[2] * x2;
    Real t11 = H[3] * x0;
    Real t12 = H[3] * x1;
    Real t13 = H[4] * x0;
    Real t14 = H[3] * x2;
    Real t15 = H[4] * x1;
    Real t16 = H[5] * x0;
    Real t17 = H[4] * x2;
    Real t18 = H[5] * x1;
    Real t19 = H[5] * x2;
    Real t20 = H[6] * x0;
    Real t21 = H[6] * x1;
    Real t22 = H[7] * x0;
    Real t23 = H[6] * x2;
    Real t24 = H[7] * x1;
    Real t25 = H[8] * x0;
    Real t26 = H[7] * x2;
    Real t27 = H[8] * x1;
    Real t28 = H[8] * x2;
    Real t32 = t2 * x1;
    Real t33 = t2 * x2;
    Real t34 = t4 * x1;
    Real t35 = t3 * x2;
    Real t36 = t4 * x2;
    Real t37 = t7 * x1;
    Real t38 = t6 * x2;
    Real t39 = t7 * x2;
    Real t40 = t9 * x2;
    Real t41 = t11 * x1;
    Real t42 = t11 * x2;
    Real t43 = t13 * x1;
    Real t44 = t12 * x2;
    Real t45 = t13 * x2;
    Real t46 = t16 * x1;
    Real t47 = t15 * x2;
    Real t48 = t16 * x2;
    Real t49 = t18 * x2;
    Real t50 = t20 * x1;
    Real t51 = t20 * x2;
    Real t52 = t22 * x1;
    Real t53 = t21 * x2;
    Real t54 = t22 * x2;
    Real t55 = t25 * x1;
    Real t56 = t24 * x2;
    Real t57 = t25 * x2;
    Real t58 = t27 * x2;
    H_affine[0] = t2 * x0;
    H_affine[1] = t32;
    H_affine[2] = t33;
    H_affine[3] = t4 * x0;
    H_affine[4] = t34;
    H_affine[5] = t36;
    H_affine[6] = t7 * x0;
    H_affine[7] = t37;
    H_affine[8] = t39;
    H_affine[9] = t2;
    H_affine[10] = t4;
    H_affine[11] = t7;
    H_affine[12] = t32;
    H_affine[13] = t3 * x1;
    H_affine[14] = t35;
    H_affine[15] = t34;
    H_affine[16] = t6 * x1;
    H_affine[17] = t38;
    H_affine[18] = t37;
    H_affine[19] = t9 * x1;
    H_affine[20] = t40;
    H_affine[21] = t3;
    H_affine[22] = t6;
    H_affine[23] = t9;
    H_affine[24] = t33;
    H_affine[25] = t35;
    H_affine[26] = t5 * x2;
    H_affine[27] = t36;
    H_affine[28] = t38;
    H_affine[29] = t8 * x2;
    H_affine[30] = t39;
    H_affine[31] = t40;
    H_affine[32] = t10 * x2;
    H_affine[33] = t5;
    H_affine[34] = t8;
    H_affine[35] = t10;
    H_affine[36] = t11 * x0;
    H_affine[37] = t41;
    H_affine[38] = t42;
    H_affine[39] = t13 * x0;
    H_affine[40] = t43;
    H_affine[41] = t45;
    H_affine[42] = t16 * x0;
    H_affine[43] = t46;
    H_affine[44] = t48;
    H_affine[45] = t11;
    H_affine[46] = t13;
    H_affine[47] = t16;
    H_affine[48] = t41;
    H_affine[49] = t12 * x1;
    H_affine[50] = t44;
    H_affine[51] = t43;
    H_affine[52] = t15 * x1;
    H_affine[53] = t47;
    H_affine[54] = t46;
    H_affine[55] = t18 * x1;
    H_affine[56] = t49;
    H_affine[57] = t12;
    H_affine[58] = t15;
    H_affine[59] = t18;
    H_affine[60] = t42;
    H_affine[61] = t44;
    H_affine[62] = t14 * x2;
    H_affine[63] = t45;
    H_affine[64] = t47;
    H_affine[65] = t17 * x2;
    H_affine[66] = t48;
    H_affine[67] = t49;
    H_affine[68] = t19 * x2;
    H_affine[69] = t14;
    H_affine[70] = t17;
    H_affine[71] = t19;
    H_affine[72] = t20 * x0;
    H_affine[73] = t50;
    H_affine[74] = t51;
    H_affine[75] = t22 * x0;
    H_affine[76] = t52;
    H_affine[77] = t54;
    H_affine[78] = t25 * x0;
    H_affine[79] = t55;
    H_affine[80] = t57;
    H_affine[81] = t20;
    H_affine[82] = t22;
    H_affine[83] = t25;
    H_affine[84] = t50;
    H_affine[85] = t21 * x1;
    H_affine[86] = t53;
    H_affine[87] = t52;
    H_affine[88] = t24 * x1;
    H_affine[89] = t56;
    H_affine[90] = t55;
    H_affine[91] = t27 * x1;
    H_affine[92] = t58;
    H_affine[93] = t21;
    H_affine[94] = t24;
    H_affine[95] = t27;
    H_affine[96] = t51;
    H_affine[97] = t53;
    H_affine[98] = t23 * x2;
    H_affine[99] = t54;
    H_affine[100] = t56;
    H_affine[101] = t26 * x2;
    H_affine[102] = t57;
    H_affine[103] = t58;
    H_affine[104] = t28 * x2;
    H_affine[105] = t23;
    H_affine[106] = t26;
    H_affine[107] = t28;
    H_affine[108] = t2;
    H_affine[109] = t3;
    H_affine[110] = t5;
    H_affine[111] = t4;
    H_affine[112] = t6;
    H_affine[113] = t8;
    H_affine[114] = t7;
    H_affine[115] = t9;
    H_affine[116] = t10;
    H_affine[117] = H[0];
    H_affine[118] = H[1];
    H_affine[119] = H[2];
    H_affine[120] = t11;
    H_affine[121] = t12;
    H_affine[122] = t14;
    H_affine[123] = t13;
    H_affine[124] = t15;
    H_affine[125] = t17;
    H_affine[126] = t16;
    H_affine[127] = t18;
    H_affine[128] = t19;
    H_affine[129] = H[3];
    H_affine[130] = H[4];
    H_affine[131] = H[5];
    H_affine[132] = t20;
    H_affine[133] = t21;
    H_affine[134] = t23;
    H_affine[135] = t22;
    H_affine[136] = t24;
    H_affine[137] = t26;
    H_affine[138] = t25;
    H_affine[139] = t27;
    H_affine[140] = t28;
    H_affine[141] = H[6];
    H_affine[142] = H[7];
    H_affine[143] = H[8];
}

// Extracted without edits: rotational/rotational_cuda.cuh : rotational_energy
template<typename Real>
__host__ __device__ __forceinline__
Real rotational_energy(Real* rest_pos0, Real* rest_pos1, Real* DOF0, Real* DOF1)
{
    Real e0[3], e1[3], diff[3];
    affine_transform<Real>(DOF0, rest_pos0, e0);
    affine_transform<Real>(DOF1, rest_pos1, e1);
    vecSubs<Real>(diff, e0, e1, 3);
    Real energy = 0.5 * dot3<Real>(diff, diff);
    return energy;
}

// Extracted without edits: rotational/rotational_cuda.cuh : rotational_gradient
template<typename Real>
__host__ __device__ __forceinline__
void rotational_gradient(Real* rest_pos0, Real* rest_pos1, Real* DOF0, Real* DOF1, Real* g)
{
    Real x00 = rest_pos0[0];
    Real x01 = rest_pos0[1];
    Real x02 = rest_pos0[2];
    Real x10 = rest_pos1[0];
    Real x11 = rest_pos1[1];
    Real x12 = rest_pos1[2];
    Real DOF00 = DOF0[0], DOF01 = DOF0[1], DOF02 = DOF0[2], DOF03 = DOF0[3], DOF04 = DOF0[4], DOF05 = DOF0[5], DOF06 = DOF0[6], DOF07 = DOF0[7], DOF08 = DOF0[8], DOF09 = DOF0[9], DOF010 = DOF0[10], DOF011 = DOF0[11];
    Real DOF10 = DOF1[0], DOF11 = DOF1[1], DOF12 = DOF1[2], DOF13 = DOF1[3], DOF14 = DOF1[4], DOF15 = DOF1[5], DOF16 = DOF1[6], DOF17 = DOF1[7], DOF18 = DOF1[8], DOF19 = DOF1[9], DOF110 = DOF1[10], DOF111 = DOF1[11];
    Real t2 = DOF00 * x00;
    Real t3 = DOF01 * x01;
    Real t4 = DOF03 * x00;
    Real t5 = DOF02 * x02;
    Real t6 = DOF04 * x01;
    Real t7 = DOF06 * x00;
    Real t8 = DOF05 * x02;
    Real t9 = DOF07 * x01;
    Real t10 = DOF08 * x02;
    Real t11 = DOF10 * x10;
    Real t12 = DOF11 * x11;
    Real t13 = DOF13 * x10;
    Real t14 = DOF12 * x12;
    Real t15 = DOF14 * x11;
    Real t16 = DOF16 * x10;
    Real t17 = DOF15 * x12;
    Real t18 = DOF17 * x11;
    Real t19 = DOF18 * x12;
    Real t20 = -DOF19;
    Real t21 = -DOF110;
    Real t22 = -DOF111;
    Real t23 = -t11;
    Real t24 = -t12;
    Real t25 = -t13;
    Real t26 = -t14;
    Real t27 = -t15;
    Real t28 = -t16;
    Real t29 = -t17;
    Real t30 = -t18;
    Real t31 = -t19;
    Real t32 = DOF09 + t2 + t3 + t5 + t20 + t23 + t24 + t26;
    Real t33 = DOF010 + t4 + t6 + t8 + t21 + t25 + t27 + t29;
    Real t34 = DOF011 + t7 + t9 + t10 + t22 + t28 + t30 + t31;
    g[0] = t32 * x00;
    g[1] = t32 * x01;
    g[2] = t32 * x02;
    g[3] = t33 * x00;
    g[4] = t33 * x01;
    g[5] = t33 * x02;
    g[6] = t34 * x00;
    g[7] = t34 * x01;
    g[8] = t34 * x02;
    g[9] = t32;
    g[10] = t33;
    g[11] = t34;
    g[12] = -t32 * x10;
    g[13] = -t32 * x11;
    g[14] = -t32 * x12;
    g[15] = -t33 * x10;
    g[16] = -t33 * x11;
    g[17] = -t33 * x12;
    g[18] = -t34 * x10;
    g[19] = -t34 * x11;
    g[20] = -t34 * x12;
    g[21] = -DOF09 + DOF19 - t2 - t3 - t5 + t11 + t12 + t14;
    g[22] = -DOF010 + DOF110 - t4 - t6 - t8 + t13 + t15 + t17;
    g[23] = -DOF011 + DOF111 - t7 - t9 - t10 + t16 + t18 + t19;
}

// Extracted without edits: rotational/rotational_cuda.cuh : rotational_Hessian
template<typename Real>
__host__ __device__ __forceinline__
void rotational_Hessian(Real* rest_pos0, Real* rest_pos1, Real* H)
{
    Real x00 = rest_pos0[0];
    Real x01 = rest_pos0[1];
    Real x02 = rest_pos0[2];
    Real x10 = rest_pos1[0];
    Real x11 = rest_pos1[1];
    Real x12 = rest_pos1[2];
    memset(H, 0, sizeof(Real) * 576);
    Real t2 = x00 * x01;
    Real t3 = x00 * x02;
    Real t4 = x01 * x02;
    Real t5 = x00 * x10;
    Real t6 = x00 * x11;
    Real t7 = x01 * x10;
    Real t8 = x00 * x12;
    Real t9 = x01 * x11;
    Real t10 = x02 * x10;
    Real t11 = x01 * x12;
    Real t12 = x02 * x11;
    Real t13 = x02 * x12;
    Real t14 = x10 * x11;
    Real t15 = x10 * x12;
    Real t16 = x11 * x12;
    Real t17 = x00 * x00;
    Real t18 = x01 * x01;
    Real t19 = x02 * x02;
    Real t20 = x10 * x10;
    Real t21 = x11 * x11;
    Real t22 = x12 * x12;
    Real t23 = -x00;
    Real t24 = -x01;
    Real t25 = -x02;
    Real t26 = -x10;
    Real t27 = -x11;
    Real t28 = -x12;
    Real t29 = -t5;
    Real t30 = -t6;
    Real t31 = -t7;
    Real t32 = -t8;
    Real t33 = -t9;
    Real t34 = -t10;
    Real t35 = -t11;
    Real t36 = -t12;
    Real t37 = -t13;
    H[0] = t17;
    H[1] = t2;
    H[2] = t3;
    H[9] = x00;
    H[12] = t29;
    H[13] = t30;
    H[14] = t32;
    H[21] = t23;
    H[24] = t2;
    H[25] = t18;
    H[26] = t4;
    H[33] = x01;
    H[36] = t31;
    H[37] = t33;
    H[38] = t35;
    H[45] = t24;
    H[48] = t3;
    H[49] = t4;
    H[50] = t19;
    H[57] = x02;
    H[60] = t34;
    H[61] = t36;
    H[62] = t37;
    H[69] = t25;
    H[75] = t17;
    H[76] = t2;
    H[77] = t3;
    H[82] = x00;
    H[87] = t29;
    H[88] = t30;
    H[89] = t32;
    H[94] = t23;
    H[99] = t2;
    H[100] = t18;
    H[101] = t4;
    H[106] = x01;
    H[111] = t31;
    H[112] = t33;
    H[113] = t35;
    H[118] = t24;
    H[123] = t3;
    H[124] = t4;
    H[125] = t19;
    H[130] = x02;
    H[135] = t34;
    H[136] = t36;
    H[137] = t37;
    H[142] = t25;
    H[150] = t17;
    H[151] = t2;
    H[152] = t3;
    H[155] = x00;
    H[162] = t29;
    H[163] = t30;
    H[164] = t32;
    H[167] = t23;
    H[174] = t2;
    H[175] = t18;
    H[176] = t4;
    H[179] = x01;
    H[186] = t31;
    H[187] = t33;
    H[188] = t35;
    H[191] = t24;
    H[198] = t3;
    H[199] = t4;
    H[200] = t19;
    H[203] = x02;
    H[210] = t34;
    H[211] = t36;
    H[212] = t37;
    H[215] = t25;
    H[216] = x00;
    H[217] = x01;
    H[218] = x02;
    H[225] = 1.0;
    H[228] = t26;
    H[229] = t27;
    H[230] = t28;
    H[237] = -1.0;
    H[243] = x00;
    H[244] = x01;
    H[245] = x02;
    H[250] = 1.0;
    H[255] = t26;
    H[256] = t27;
    H[257] = t28;
    H[262] = -1.0;
    H[270] = x00;
    H[271] = x01;
    H[272] = x02;
    H[275] = 1.0;
    H[282] = t26;
    H[283] = t27;
    H[284] = t28;
    H[287] = -1.0;
    H[288] = t29;
    H[289] = t31;
    H[290] = t34;
    H[297] = t26;
    H[300] = t20;
    H[301] = t14;
    H[302] = t15;
    H[309] = x10;
    H[312] = t30;
    H[313] = t33;
    H[314] = t36;
    H[321] = t27;
    H[324] = t14;
    H[325] = t21;
    H[326] = t16;
    H[333] = x11;
    H[336] = t32;
    H[337] = t35;
    H[338] = t37;
    H[345] = t28;
    H[348] = t15;
    H[349] = t16;
    H[350] = t22;
    H[357] = x12;
    H[363] = t29;
    H[364] = t31;
    H[365] = t34;
    H[370] = t26;
    H[375] = t20;
    H[376] = t14;
    H[377] = t15;
    H[382] = x10;
    H[387] = t30;
    H[388] = t33;
    H[389] = t36;
    H[394] = t27;
    H[399] = t14;
    H[400] = t21;
    H[401] = t16;
    H[406] = x11;
    H[411] = t32;
    H[412] = t35;
    H[413] = t37;
    H[418] = t28;
    H[423] = t15;
    H[424] = t16;
    H[425] = t22;
    H[430] = x12;
    H[438] = t29;
    H[439] = t31;
    H[440] = t34;
    H[443] = t26;
    H[450] = t20;
    H[451] = t14;
    H[452] = t15;
    H[455] = x10;
    H[462] = t30;
    H[463] = t33;
    H[464] = t36;
    H[467] = t27;
    H[474] = t14;
    H[475] = t21;
    H[476] = t16;
    H[479] = x11;
    H[486] = t32;
    H[487] = t35;
    H[488] = t37;
    H[491] = t28;
    H[498] = t15;
    H[499] = t16;
    H[500] = t22;
    H[503] = x12;
    H[504] = t23;
    H[505] = t24;
    H[506] = t25;
    H[513] = -1.0;
    H[516] = x10;
    H[517] = x11;
    H[518] = x12;
    H[525] = 1.0;
    H[531] = t23;
    H[532] = t24;
    H[533] = t25;
    H[538] = -1.0;
    H[543] = x10;
    H[544] = x11;
    H[545] = x12;
    H[550] = 1.0;
    H[558] = t23;
    H[559] = t24;
    H[560] = t25;
    H[563] = -1.0;
    H[570] = x10;
    H[571] = x11;
    H[572] = x12;
    H[575] = 1.0;
}

// Extracted without edits: sparse_matrix_square/spmv_cuda.cuh : spmv_sm_kernel
template<typename Real, unsigned int block_size>
__global__ void spmv_sm_kernel(unsigned int nnz, unsigned int* blocks_to_coords, Real* nonzeros, Real* vec, Real* result)
{
    unsigned int global_warp_id = (blockDim.x * blockIdx.x + threadIdx.x) / 32;
    if (global_warp_id >= nnz) return;

    auto lane_id = threadIdx.x % 32;
    unsigned int b = global_warp_id;

    unsigned int row_idx = blocks_to_coords[2 * b];
    unsigned int col_idx = blocks_to_coords[2 * b + 1];

    Real sum = 0;
    if (lane_id < block_size)
	{

        for (unsigned int j = 0; j < block_size; ++j)
		{	
            sum += nonzeros[b * block_size * block_size + lane_id * block_size + j] * vec[col_idx * block_size + j];
        }
        
        atomicAdd(&result[row_idx * block_size + lane_id], sum);
    }
}

// Extracted without edits: sparse_linear_solvers/conjugate_gradient_cuda.cuh : dmv_accumulate_kernel
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

extern "C" __global__ void validate_affine(double* rest, double* g, double* h,
                                         double* p, double* pg, double* ph) {
    affine_projection<double>(rest, p);
    affine_projection_gradient3x1<double>(rest, g, pg);
    affine_projection_Hessian3x3<double>(rest, h, ph);
}
extern "C" __global__ void validate_rotational(double* r0, double* r1,
    double* q0, double* q1, double* e, double* g, double* h) {
    e[0] = rotational_energy<double>(r0, r1, q0, q1);
    rotational_gradient<double>(r0, r1, q0, q1, g);
    rotational_Hessian<double>(r0, r1, h);
}
