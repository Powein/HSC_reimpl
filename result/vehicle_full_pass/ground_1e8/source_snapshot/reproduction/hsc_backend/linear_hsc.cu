// HSC integration for libuipc 9c748a759bb9952a7aeedd59076a70591dc61d6c.
// Implements HC.pdf Eqs. (2)-(6), supplement Algorithm 1. New implementation.
// Sparse FEM iteration stays on CUDA; small ABD/ACA setup uses Eigen on the CPU.
// This hybrid implementation is not the authors' GPU performance implementation.
#include <linear_system/iterative_solver.h>
#include <affine_body/abd_linear_subsystem.h>
#include <finite_element/fem_linear_subsystem.h>
#include <collision_detection/simplex_trajectory_filter.h>
#include <collision_detection/vertex_half_plane_trajectory_filter.h>
#include <sim_engine.h>
#include <cuda_tool/linear_reduction.h>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <vector>

namespace uipc::backend::cuda
{
namespace hsc
{
using CV = cuda_tool::CDenseVectorView<Float>;
using V = cuda_tool::DenseVectorView<Float>;
using CM = cuda_tool::CBCOOMatrixView<Float, 3>;
using Mat = Eigen::Matrix<Float, Eigen::Dynamic, Eigen::Dynamic>;
using Vec = Eigen::Matrix<Float, Eigen::Dynamic, 1>;
using M3 = Eigen::Matrix<Float, 3, 3>;
using DV = cuda_tool::DeviceDenseVector<Float>;

void check(cudaError_t e)
{
    if(e != cudaSuccess) throw std::runtime_error(cudaGetErrorString(e));
}
template<class T> struct GPU
{
    T* data = nullptr;
    size_t capacity = 0;
    ~GPU() { if(data) cudaFree(data); }
    void resize(size_t n)
    {
        if(n <= capacity) return;
        if(data) check(cudaFree(data));
        data = nullptr; capacity = 0;
        check(cudaMalloc(reinterpret_cast<void**>(&data), n*sizeof(T)));
        capacity = n;
    }
    void upload(const T* p, size_t n)
    {
        resize(n);
        if(n) check(cudaMemcpy(data, p, n*sizeof(T), cudaMemcpyHostToDevice));
    }
};
struct Entry { int row, col; Float v[9]; };

// libuipc stores ONE triangle of 3x3 blocks after ge2sym(). Do not double
// diagonal blocks, and transpose off-diagonal blocks when reconstructing A/E.
__global__ void select(CM H, int qo, int qm, int fo, int fn,
                       int* ids, int* counters, M3* inv)
{
    int t = blockIdx.x*blockDim.x+threadIdx.x;
    if(t >= H.triplet_count()) return;
    auto e = H(t);
    int i=e.row_index*3, j=e.col_index*3;
    bool iq=i>=qo && i<qo+qm, jq=j>=qo && j<qo+qm;
    if(iq || jq) ids[atomicAdd(counters,1)] = t;
    if(i==j && i>=fo && i<fo+fn)
    {
        M3 B=e.value;
        Float det=B.determinant();
        if(!(det>0) || !isfinite(det))
        {
            atomicAdd(counters+1,1);
            inv[i/3].setZero();
        }
        else inv[i/3]=B.inverse();
    }
}
__global__ void pack(CM H, const int* ids, Entry* entries, int n)
{
    int t=blockIdx.x*blockDim.x+threadIdx.x;
    if(t>=n) return;
    auto e=H(ids[t]);
    entries[t].row=e.row_index*3;
    entries[t].col=e.col_index*3;
    for(int c=0;c<3;++c) for(int r=0;r<3;++r)
        entries[t].v[r+3*c]=e.value(r,c);
}
__global__ void mask_q(V v,int off,int n)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i<n) v(off+i)=0;
}
__global__ void jacobi(const M3* inv,CV r,V z,int fo,int fn,Float omega)
{
    int j=blockIdx.x*blockDim.x+threadIdx.x;
    if(j>=fn/3) return;
    int i=fo+3*j;
    Eigen::Matrix<Float,3,1> b;
    for(int k=0;k<3;++k) b(k)=r(i+k);
    Eigen::Matrix<Float,3,1> a=omega*inv[i/3]*b;
    for(int k=0;k<3;++k) z(i+k)=a(k);
}
__global__ void axpby(V y,CV x,Float a,Float b,int n)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i<n) y(i)=a*x(i)+b*y(i);
}
__global__ void update(V x,V r,CV p,CV Ap,Float alpha,int n)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i<n) { x(i)+=alpha*p(i); r(i)-=alpha*Ap(i); }
}
__global__ void transpose_V(const Float* factor,const int* nodes,int n,
                            CV x,Float* out)
{
    int k=blockIdx.x;
    __shared__ Float sums[256];
    Float sum=0;
    for(int j=threadIdx.x;j<n;j+=256) sum+=factor[j+k*n]*x(nodes[j]);
    sums[threadIdx.x]=sum;
    __syncthreads();
    for(int stride=128;stride>0;stride>>=1)
    {
        if(threadIdx.x<stride) sums[threadIdx.x]+=sums[threadIdx.x+stride];
        __syncthreads();
    }
    if(threadIdx.x==0) out[k]=sums[0];
}
__global__ void small_product(const Float* C,const Float* x,Float* y,int k)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=k) return;
    Float sum=0;
    for(int j=0;j<k;++j) sum+=C[i+j*k]*x[j];
    y[i]=sum;
}
__global__ void subtract_V(const Float* factor,const int* nodes,int n,int k,
                           const Float* c,V y)
{
    int i=blockIdx.x*blockDim.x+threadIdx.x;
    if(i>=n) return;
    Float sum=0;
    for(int j=0;j<k;++j) sum+=factor[i+j*n]*c[j];
    y(nodes[i])-=sum;
}
int blocks(int n) { return (n+255)/256; }
}

class LinearHSC final : public IterativeSolver
{
  public:
    using IterativeSolver::IterativeSolver;
  private:
    ABDLinearSubsystem* abd=nullptr;
    FEMLinearSubsystem* fem=nullptr;
    GlobalTrajectoryFilter* contacts=nullptr;
    int qo=0,qm=0,fo=0,fn=0,N=0,rank=0,active=0;
    int max_rank=128,order=5,max_iter=2000,adaptive_abd=1,doublings=0;
    Float omega=.7,tol=1e-4,aca_error=0,abd_inverse_error=0;
    Float abd_inverse_tol=1e-5,full_residual_limit=.1;
    hsc::GPU<int> ids,counters,nodes;
    hsc::GPU<hsc::Entry> packed;
    hsc::GPU<hsc::M3> invD;
    hsc::GPU<Float> dV,dC,t1,t2;
    hsc::Mat A,P,U,V;
    hsc::DV r,z,p,Ap,term,tmp,aux,rhs;

    void do_build(BuildInfo&) override
    {
        auto& cfg=world().scene().config();
        auto attr=cfg.find<std::string>("linear_system/solver");
        if(!attr || attr->view()[0]!="linear_hsc") throw SimSystemException("LinearHSC unused");
        abd=&require<ABDLinearSubsystem>();
        fem=&require<FEMLinearSubsystem>();
        contacts=&require<GlobalTrajectoryFilter>();
        max_rank=cfg.find<IndexT>("linear_system/hsc/max_rank")->view()[0];
        order=cfg.find<IndexT>("linear_system/hsc/neumann_order")->view()[0];
        max_iter=cfg.find<IndexT>("linear_system/hsc/max_iter")->view()[0];
        omega=cfg.find<Float>("linear_system/hsc/omega")->view()[0];
        adaptive_abd=cfg.find<IndexT>("linear_system/hsc/adaptive_abd_inverse")->view()[0];
        abd_inverse_tol=cfg.find<Float>("linear_system/hsc/abd_inverse_tol")->view()[0];
        full_residual_limit=cfg.find<Float>("linear_system/hsc/full_residual_limit")->view()[0];
        tol=cfg.find<Float>("linear_system/tol_rate")->view()[0];
        UIPC_ASSERT(max_rank>0 && order>0 && max_iter>0 && omega>0 && omega<=1,
                    "Invalid HSC parameters");
        logger::info("LinearHSC enabled: rank_cap={}, Neumann order={}, omega={}, tol={}. "
                     "CUDA FEM/Schur PCG; CPU small ABD/ACA setup; no solver fallback.",
                     max_rank,order,omega,tol);
    }

    void setup(GlobalLinearSystem::SolvingInfo& info)
    {
        N=info.x().size(); qo=abd->dof_offset(); qm=abd->dof_count();
        fo=fem->dof_offset(); fn=fem->dof_count();
        UIPC_ASSERT(qm>0 && fn>0 && qm%12==0 && fn%3==0 && qm+fn==N,
                    "HSC requires only ABD (12/body) and FEM (3/node) DOFs");
        auto H=info.A();
        ids.resize(H.triplet_count()); counters.resize(2); invD.resize(N/3);
        hsc::check(cudaMemset(counters.data,0,2*sizeof(int)));
        hsc::check(cudaMemset(invD.data,0,N/3*sizeof(hsc::M3)));
        hsc::select<<<hsc::blocks(H.triplet_count()),256>>>(H,qo,qm,fo,fn,ids.data,counters.data,invD.data);
        int count[2];
        hsc::check(cudaMemcpy(count,counters.data,sizeof(count),cudaMemcpyDeviceToHost));
        UIPC_ASSERT(count[1]==0,"HSC: {} non-positive FEM diagonal blocks",count[1]);
        packed.resize(count[0]);
        if(count[0]) hsc::pack<<<hsc::blocks(count[0]),256>>>(H,ids.data,packed.data,count[0]);
        std::vector<hsc::Entry> entries(count[0]);
        if(count[0]) hsc::check(cudaMemcpy(entries.data(),packed.data,count[0]*sizeof(hsc::Entry),cudaMemcpyDeviceToHost));
        // Stable ordering makes full-pivot ACA independent of atomic compaction order.
        std::sort(entries.begin(),entries.end(),[](const auto& a,const auto& b)
                  { return a.row==b.row?a.col<b.col:a.row<b.row; });
        std::vector<int> cols;
        A=hsc::Mat::Zero(qm,qm);
        for(auto& e:entries)
        {
            bool iq=e.row>=qo && e.row<qo+qm, jq=e.col>=qo && e.col<qo+qm;
            Eigen::Map<const hsc::M3> B(e.v);
            if(iq && jq)
            {
                A.block<3,3>(e.row-qo,e.col-qo)+=B;
                if(e.row!=e.col) A.block<3,3>(e.col-qo,e.row-qo)+=B.transpose();
            }
            else if(B.squaredNorm()!=0)
            {
                int f=iq?e.col:e.row;
                UIPC_ASSERT(f>=fo && f<fo+fn,"Unexpected HSC subsystem coupling");
                for(int j=0;j<3;++j) cols.push_back(f+j);
            }
        }
        std::sort(cols.begin(),cols.end()); cols.erase(std::unique(cols.begin(),cols.end()),cols.end());
        active=static_cast<int>(cols.size());
        nodes.upload(cols.data(),active);
        hsc::Mat E=hsc::Mat::Zero(qm,active);
        for(auto& e:entries)
        {
            bool iq=e.row>=qo && e.row<qo+qm, jq=e.col>=qo && e.col<qo+qm;
            Eigen::Map<const hsc::M3> B(e.v);
            if(iq==jq || B.squaredNorm()==0) continue;
            int q=(iq?e.row:e.col)-qo, f=iq?e.col:e.row;
            int c=static_cast<int>(std::lower_bound(cols.begin(),cols.end(),f)-cols.begin());
            if(iq) E.block<3,3>(q,c)+=B;
            else E.block<3,3>(q,c)+=B.transpose();
        }
        // Full-pivot ACA with a fixed rank cap, as in the supplement. Only a
        // roundoff zero-pivot guard may stop early. No SVD truncation or rtol.
        int cap=std::min({max_rank,qm,active});
        U=hsc::Mat::Zero(qm,cap); V=hsc::Mat::Zero(active,cap);
        hsc::Mat R=E;
        Float scale=E.norm(); rank=0;
        for(;rank<cap;++rank)
        {
            Eigen::Index i,j; Float peak=R.cwiseAbs().maxCoeff(&i,&j);
            if(peak<=std::numeric_limits<Float>::epsilon()*scale) break;
            U.col(rank)=R.col(j); V.col(rank)=R.row(i).transpose()/R(i,j);
            R.noalias()-=U.col(rank)*V.col(rank).transpose();
        }
        U.conservativeResize(Eigen::NoChange,rank); V.conservativeResize(Eigen::NoChange,rank);
        aca_error=scale>0?R.norm()/scale:0;
        // Block-Jacobi Neumann approximation of A^-1 (12 DOFs/body).
        hsc::Mat B=hsc::Mat::Zero(qm,qm);
        for(int i=0;i<qm;i+=12)
        {
            Eigen::Matrix<Float,12,12> block=A.block<12,12>(i,i);
            Eigen::LLT<Eigen::Matrix<Float,12,12>> llt(block);
            UIPC_ASSERT(llt.info()==Eigen::Success,"HSC: non-SPD ABD block {}",i/12);
            B.block<12,12>(i,i)=llt.solve(Eigen::Matrix<Float,12,12>::Identity());
        }
        hsc::Mat T=hsc::Mat::Identity(qm,qm)-omega*B*A;
        hsc::Mat termA=omega*B; P=termA;
        for(int k=1;k<order;++k) { termA=(T*termA).eval(); P+=termA; }
        P=((P+P.transpose())*.5).eval();
        // The paper's fixed five terms can miss nearly free collective modes
        // of a vehicle constrained by very stiff joints. Extend the SAME
        // Neumann polynomial by doubling its length, P_2r=P_r+P_r(I-A P_r).
        // This is an explicit robustness extension, not the fixed-r paper run
        // and not a fallback to another/global solver. FEM keeps five terms.
        const hsc::Mat I=hsc::Mat::Identity(qm,qm);
        hsc::Mat inverse_residual=I-A*P;
        abd_inverse_error=inverse_residual.norm()/std::sqrt(Float(qm));
        doublings=0;
        while(adaptive_abd && abd_inverse_error>abd_inverse_tol && doublings<48)
        {
            P=(P+P*inverse_residual).eval();
            P=((P+P.transpose())*.5).eval();
            inverse_residual=I-A*P;
            abd_inverse_error=inverse_residual.norm()/std::sqrt(Float(qm));
            ++doublings;
            UIPC_ASSERT(P.allFinite() && std::isfinite(abd_inverse_error),
                        "HSC ABD Neumann extension diverged at doubling {}",doublings);
        }
        UIPC_ASSERT(!adaptive_abd || abd_inverse_error<=abd_inverse_tol,
                    "HSC ABD inverse residual {} exceeds {} after {} doublings",
                    abd_inverse_error,abd_inverse_tol,doublings);
        hsc::Mat C=U.transpose()*P*U;
        C=((C+C.transpose())*.5).eval();
        UIPC_ASSERT(P.allFinite() && C.allFinite(),"HSC: nonfinite Neumann/Schur factors");
        dV.upload(V.data(),V.size()); dC.upload(C.data(),C.size());
        t1.resize(rank); t2.resize(rank);
        for(auto v:{&r,&z,&p,&Ap,&term,&tmp,&aux,&rhs}) v->resize(N);
    }

    void D_product(hsc::CV x,hsc::V y)
    {
        // x has zero ABD entries. Masking H*x therefore gives exactly D*x.
        spmv(x,y);
        hsc::mask_q<<<hsc::blocks(qm),256>>>(y,qo,qm);
    }
    void VT(hsc::CV x)
    {
        if(rank) hsc::transpose_V<<<rank,256>>>(dV.data,nodes.data,active,x,t1.data);
    }
    void subtract_factor(hsc::V y,const Float* c)
    {
        if(rank) hsc::subtract_V<<<hsc::blocks(active),256>>>(dV.data,nodes.data,active,rank,c,y);
    }
    void schur(hsc::CV x,hsc::V y)
    {
        D_product(x,y);
        if(rank)
        {
            VT(x);
            hsc::small_product<<<hsc::blocks(rank),256>>>(dC.data,t1.data,t2.data,rank);
            subtract_factor(y,t2.data);
        }
    }
    void precondition(hsc::CV b)
    {
        z.buffer_view().fill(0); term.buffer_view().fill(0);
        hsc::jacobi<<<hsc::blocks(fn/3),256>>>(invD.data,b,z.view(),fo,fn,omega);
        term.buffer_view().copy_from(z.cview().buffer_view());
        for(int k=1;k<order;++k)
        {
            D_product(term.cview(),tmp.view());
            aux.buffer_view().fill(0);
            hsc::jacobi<<<hsc::blocks(fn/3),256>>>(invD.data,tmp.cview(),aux.view(),fo,fn,omega);
            hsc::axpby<<<hsc::blocks(N),256>>>(term.view(),aux.cview(),-1,1,N);
            hsc::axpby<<<hsc::blocks(N),256>>>(z.view(),term.cview(),1,1,N);
        }
    }
    void do_solve(GlobalLinearSystem::SolvingInfo& info) override
    {
        auto start=std::chrono::steady_clock::now();
        setup(info);
        auto prepared=std::chrono::steady_clock::now();
        auto x=info.x(); auto b=info.b();
        x.buffer_view().fill(0);
        hsc::Vec bq(qm);
        hsc::check(cudaMemcpy(bq.data(),b.buffer_view().data()+qo,qm*sizeof(Float),cudaMemcpyDeviceToHost));
        hsc::Vec c=U.transpose()*P*bq;
        t2.upload(c.data(),rank);
        rhs.buffer_view().copy_from(b.buffer_view());
        hsc::mask_q<<<hsc::blocks(qm),256>>>(rhs.view(),qo,qm);
        subtract_factor(rhs.view(),t2.data);
        r.buffer_view().copy_from(rhs.cview().buffer_view());
        Float normb=ctx().norm(rhs.cview()),relative=normb>0?1:0;
        Float rz=0; int iterations=0;
        if(normb>0)
        {
            precondition(r.cview());
            p.buffer_view().copy_from(z.cview().buffer_view());
            rz=ctx().dot(r.cview(),z.cview());
            while(iterations<max_iter && relative>tol)
            {
                schur(p.cview(),Ap.view());
                Float denom=ctx().dot(p.cview(),Ap.cview());
                UIPC_ASSERT(std::isfinite(denom) && denom>0 && std::isfinite(rz) && rz>0,
                            "HSC Schur PCG breakdown: iteration={}, pSp={}, rz={}. No fallback.",iterations,denom,rz);
                hsc::update<<<hsc::blocks(N),256>>>(x,r.view(),p.cview(),Ap.cview(),rz/denom,N);
                ++iterations;
                relative=ctx().norm(r.cview())/normb;
                if(relative<=tol)
                {
                    // One true residual check on convergence, not a per-iteration audit.
                    schur(x.as_const(),Ap.view());
                    r.buffer_view().copy_from(rhs.cview().buffer_view());
                    hsc::axpby<<<hsc::blocks(N),256>>>(r.view(),Ap.cview(),-1,1,N);
                    relative=ctx().norm(r.cview())/normb;
                    if(relative<=tol) break;
                    precondition(r.cview());
                    p.buffer_view().copy_from(z.cview().buffer_view());
                    rz=ctx().dot(r.cview(),z.cview());
                    continue;
                }
                precondition(r.cview());
                Float next=ctx().dot(r.cview(),z.cview());
                hsc::axpby<<<hsc::blocks(N),256>>>(p.view(),z.cview(),1,next/rz,N);
                rz=next;
            }
        }
        UIPC_ASSERT(relative<=tol,"HSC Schur PCG did not converge: iterations={}, residual={}",iterations,relative);
        VT(x.as_const());
        hsc::Vec vt=hsc::Vec::Zero(rank);
        if(rank) hsc::check(cudaMemcpy(vt.data(),t1.data,rank*sizeof(Float),cudaMemcpyDeviceToHost));
        hsc::Vec dq=P*(bq-U*vt);
        hsc::check(cudaMemcpy(x.buffer_view().data()+qo,dq.data(),qm*sizeof(Float),cudaMemcpyHostToDevice));
        Float descent=ctx().dot(b,x.as_const());
        Float bn=ctx().norm(b);
        UIPC_ASSERT(std::isfinite(descent) && (descent>0 || bn==0),"HSC returned a non-descent direction");
        spmv(x.as_const(),Ap.view());
        r.buffer_view().copy_from(b.buffer_view());
        hsc::axpby<<<hsc::blocks(N),256>>>(r.view(),Ap.cview(),-1,1,N);
        Float full_residual=bn>0?ctx().norm(r.cview())/bn:0;
        auto end=std::chrono::steady_clock::now();
        double setup_ms=std::chrono::duration<double,std::milli>(prepared-start).count();
        double total_ms=std::chrono::duration<double,std::milli>(end-start).count();
        SizeT mesh_contacts=0,ground_contacts=0;
        auto sf=contacts->find<SimplexTrajectoryFilter>();
        auto gf=contacts->find<VertexHalfPlaneTrajectoryFilter>();
        if(sf) mesh_contacts=sf->PTs().size()+sf->EEs().size()+sf->PEs().size()+sf->PPs().size();
        if(gf) ground_contacts=gf->PHs().size();
        info.iter_count(iterations);
        logger::info("LinearHSC: frame={} newton={} FEM={} ABD={} active={} rank={} ACAerr={} "
                     "iters={} SchurResidual={} fullResidual={} setup_ms={} total_ms={}",
                     engine().frame(),engine().newton_iter(),fn,qm,active,rank,aca_error,
                     iterations,relative,full_residual,setup_ms,total_ms);
        std::ofstream out(std::filesystem::path(workspace())/"hsc_stats.jsonl",std::ios::app);
        out<<fmt::format("{{\"frame\":{},\"newton\":{},\"fem_dofs\":{},\"abd_dofs\":{},"
                         "\"active_fem_dofs\":{},\"rank\":{},\"aca_relative_error\":{},"
                         "\"pcg_iterations\":{},\"schur_relative_residual\":{},\"full_relative_residual\":{},"
                         "\"setup_ms\":{},\"total_ms\":{},\"active_mesh_contacts\":{},\"active_ground_contacts\":{},"
                         "\"adaptive_abd_inverse\":{},\"neumann_doublings\":{},\"abd_inverse_residual\":{},"
                         "\"full_residual_limit\":{},\"accepted\":{},"
                         "\"solver\":\"linear_hsc\",\"fallback\":false}}\n",
                         engine().frame(),engine().newton_iter(),fn,qm,active,rank,aca_error,
                         iterations,relative,full_residual,setup_ms,total_ms,mesh_contacts,ground_contacts,
                         bool(adaptive_abd),doublings,abd_inverse_error,full_residual_limit,
                         std::isfinite(full_residual) && full_residual<=full_residual_limit);
        out.flush();
        UIPC_ASSERT(std::isfinite(full_residual) && full_residual<=full_residual_limit,
                    "HSC full-system residual {} exceeds {} despite Schur residual {}. No fallback.",
                    full_residual,full_residual_limit,relative);
    }
};
REGISTER_SIM_SYSTEM(LinearHSC);
} // namespace uipc::backend::cuda
