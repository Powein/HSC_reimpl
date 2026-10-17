// One numerical check of the same header compiled into the CUDA constitution.
#include <finite_element/constitutions/log_neo_hookean_function.h>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace uipc;
using namespace uipc::backend::cuda;

__global__ void device_check(double* values, int stage)
{
    Matrix3x3 F=Matrix3x3::Identity(), P;
    Matrix9x9 H;
    if(stage==0) { log_nh::dEdF(P,3.2,4.7,F); values[0]=P.norm(); }
    if(stage==1) { log_nh::raw_hessian(H,3.2,4.7,F); values[0]=H.trace(); }
    if(stage==2) { log_nh::ddEddF_spd(H,3.2,4.7,F); values[0]=H.trace(); }
    if(stage==3) {
        Matrix3x3 B=F; B(0,0)=-3; B(1,1)=-1;
        values[1]=log_nh::feasible_segment(F,.25*F) && !log_nh::feasible_segment(F,B);
    }
    if(stage==4) { F(2,2)=-1; log_nh::E(values[2],3.2,4.7,F); }
}

int main()
{
    double *device_values, host_values[3];
    if(cudaMalloc(&device_values,3*sizeof(double))!=cudaSuccess) return 1;
    for(int stage=0; stage<5; ++stage) {
        device_check<<<1,1>>>(device_values,stage);
        const auto device_error=cudaDeviceSynchronize();
        if(device_error!=cudaSuccess) {
            std::cerr << "CUDA stage " << stage << " failed: " << cudaGetErrorString(device_error) << '\n';
            return 1;
        }
    }
    cudaMemcpy(host_values,device_values,3*sizeof(double),cudaMemcpyDeviceToHost);
    cudaFree(device_values);
    if(!std::isfinite(host_values[0]) || host_values[0]<=0 || host_values[1]!=1
       || !std::isinf(host_values[2]) || host_values[2]<0)
        throw std::runtime_error("Device constitutive check failed");
    std::mt19937 rng(718);
    std::normal_distribution<double> normal(0, .12);
    double max_g=0, max_h=0;
    const double mu=3.2, lambda=4.7, h=1e-6;
    for(int sample=0; sample<32; ++sample)
    {
        Matrix3x3 F=Matrix3x3::Identity();
        for(int i=0; i<9; ++i) F.data()[i]+=normal(rng);
        if(sample==0) F*=.35; // strong but feasible compression
        Matrix3x3 P;
        Matrix9x9 H;
        log_nh::dEdF(P,mu,lambda,F);
        log_nh::raw_hessian(H,mu,lambda,F);
        for(int k=0; k<9; ++k)
        {
            Matrix3x3 plus=F, minus=F, pp, pm;
            plus.data()[k]+=h; minus.data()[k]-=h;
            Float ep,em;
            log_nh::E(ep,mu,lambda,plus); log_nh::E(em,mu,lambda,minus);
            max_g=std::max(max_g,std::abs((ep-em)/(2*h)-P.data()[k])/(1+P.norm()));
            log_nh::dEdF(pp,mu,lambda,plus); log_nh::dEdF(pm,mu,lambda,minus);
            // Keep the matrix expression alive while mapping its storage.
            const Matrix3x3 diff=(pp-pm)/(2*h);
            const Eigen::Map<const Vector<Float,9>> column(diff.data());
            max_h=std::max(max_h,(column-H.col(k)).norm()/(1+H.norm()));
        }
        log_nh::ddEddF_spd(H,mu,lambda,F);
        if(Eigen::SelfAdjointEigenSolver<Matrix9x9>(H).eigenvalues().minCoeff() < -1e-8)
            throw std::runtime_error("Projected Hessian is indefinite");
        Matrix9x9 raw;
        log_nh::raw_hessian(raw,mu,lambda,F);
        Eigen::SelfAdjointEigenSolver<Matrix9x9> eig(raw);
        const Matrix9x9 expected=eig.eigenvectors()*eig.eigenvalues().cwiseMax(0.).asDiagonal()*eig.eigenvectors().transpose();
        if((H-expected).norm()/(1+expected.norm())>1e-9)
            throw std::runtime_error("Jacobi projection mismatch");
    }
    Matrix3x3 I=Matrix3x3::Identity(), B=I;
    B(0,0)=-3; B(1,1)=-1;
    // Positive determinants at both ends, negative between roots 1/4 and 1/2.
    if(log_nh::feasible_segment(I,B)) throw std::runtime_error("Skipped two roots");
    B=I; B(0,0)=-1; B(1,1)=-1;
    if(log_nh::feasible_segment(I,B)) throw std::runtime_error("Missed a double root");
    B=I; B(2,2)=-.1;
    if(log_nh::feasible_segment(I,B)) throw std::runtime_error("Accepted inverted endpoint");
    if(!log_nh::feasible_segment(I,.25*I)) throw std::runtime_error("Rejected valid compression");
    if(!log_nh::feasible_segment(I,I)) throw std::runtime_error("Rejected zero step");
    if(max_g>1e-7 || max_h>1e-7) throw std::runtime_error("Derivative mismatch");
    std::cout << "{\"samples\":32,\"max_gradient_error\":" << max_g
              << ",\"max_hessian_error\":" << max_h
              << ",\"segment_checks\":5,\"device_check\":true,\"passed\":true}\n";
}
