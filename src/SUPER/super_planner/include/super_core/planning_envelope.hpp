#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

namespace super_planner {
// Profile boxes are a UNION, never their enclosing AABB. All clients use the
// same fixed WORLD -> PROFILE transform as the mission supervisor.
struct PlanningEnvelope {
    bool enabled=false,ready=false;
    double inset=.03,world_floor=-1e6,world_ceiling=1e6;
    double profile_floor=-1e6,profile_ceiling=1e6;
    Eigen::Matrix3d rotation=Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation=Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> lower,upper;
    Eigen::Vector3d profile(const Eigen::Vector3d& p)const{return rotation*p+translation;}
    bool inBox(const Eigen::Vector3d& p,size_t i)const{
        const auto q=profile(p);
        Eigen::Vector3d lo=lower[i]+Eigen::Vector3d::Constant(inset);
        Eigen::Vector3d hi=upper[i]-Eigen::Vector3d::Constant(inset);
        lo.z()=std::max(lo.z(),profile_floor+inset);
        hi.z()=std::min(hi.z(),profile_ceiling-inset);
        return p.allFinite() && p.z()>=world_floor && p.z()<=world_ceiling &&
            (q.array()>=lo.array()).all() && (q.array()<=hi.array()).all();
    }
    bool contains(const Eigen::Vector3d& p)const{
        if(!enabled)return true;
        if(!ready)return false;
        for(size_t i=0;i<lower.size();++i)if(inBox(p,i))return true;
        return false;
    }
    int commonBox(const Eigen::Vector3d& a,const Eigen::Vector3d& b)const{
        if(!enabled)return -2;
        if(!ready)return -1;
        int best=-1;double room=-1.;
        for(size_t i=0;i<lower.size();++i)if(inBox(a,i) && inBox(b,i)){
            const auto x=profile(a),y=profile(b);
            const double d=std::min((x-lower[i]).minCoeff(),std::min((upper[i]-x).minCoeff(),
                std::min((y-lower[i]).minCoeff(),(upper[i]-y).minCoeff())));
            if(d>room){room=d;best=int(i);}
        }
        return best;
    }
    // Clip a convex corridor to one member of the union. Seed selection splits
    // at box transitions, so no convex corridor bridges a forbidden corner.
    bool clip(Eigen::Matrix<double,Eigen::Dynamic,4>& planes,
              const Eigen::Vector3d& a,const Eigen::Vector3d& b)const{
        if(!enabled)return true;
        const int box=commonBox(a,b);if(box<0)return false;
        Eigen::Vector3d lo=lower[box]+Eigen::Vector3d::Constant(inset);
        Eigen::Vector3d hi=upper[box]-Eigen::Vector3d::Constant(inset);
        lo.z()=std::max(lo.z(),profile_floor+inset);
        hi.z()=std::min(hi.z(),profile_ceiling-inset);
        const auto n=planes.rows();planes.conservativeResize(n+6,4);
        for(int k=0;k<3;++k){
            planes.block<1,3>(n+2*k,0)=rotation.row(k);
            planes(n+2*k,3)=translation[k]-hi[k];
            planes.block<1,3>(n+2*k+1,0)=-rotation.row(k);
            planes(n+2*k+1,3)=lo[k]-translation[k];
        }
        return true;
    }
};
// Enumerate actual inflated-grid centres, ranked by distance to the ORIGINAL
// request. Never clamp a cell centre or use a cell outside the mission.
template<class Position,class Allowed>
bool boundedGridGoal(const Eigen::Vector3d& request,const Eigen::Vector3i& centre,
                     double resolution,double radius,Position position,Allowed allowed,
                     Eigen::Vector3d& result){
    double best=std::numeric_limits<double>::infinity();bool found=false;
    const int n=int(std::ceil(radius/resolution))+1;
    for(int x=-n;x<=n;++x)for(int y=-n;y<=n;++y)for(int z=-n;z<=n;++z){
        const Eigen::Vector3d p=position(centre+Eigen::Vector3i(x,y,z));
        const double d=(p-request).squaredNorm();
        if(d>radius*radius+1e-12 || d>=best || !allowed(p))continue;
        result=p;best=d;found=true;
    }
    return found;
}
}
