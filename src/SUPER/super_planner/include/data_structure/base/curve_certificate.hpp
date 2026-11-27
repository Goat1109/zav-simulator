#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace super_utils {
// Bernstein convex-hull certificates cover the entire continuous polynomial,
// including maxima between samples. A finite budget fails closed.
inline bool normHullInside(const std::vector<Eigen::Vector3d>& c,double limit,int& budget,int depth=0){
    if(--budget<0 || c.empty() || !std::isfinite(limit) || limit<=0.)return false;
    bool inside=true;
    for(const auto& p:c){if(!p.allFinite())return false;inside=inside && p.norm()<=limit+1e-6;}
    if(inside)return true;
    if(c.front().norm()>limit+1e-6 || c.back().norm()>limit+1e-6 || depth>=18)return false;
    auto work=c,left=c,right=c;const size_t n=c.size();
    for(size_t level=1;level<n;++level){
        for(size_t i=0;i<n-level;++i)work[i]=.5*(work[i]+work[i+1]);
        left[level]=work[0];right[n-1-level]=work[n-1-level];
    }
    return normHullInside(left,limit,budget,depth+1) && normHullInside(right,limit,budget,depth+1);
}
template<class Matrix>
std::vector<Eigen::Vector3d> positionControls(const Matrix& c,double duration){
    std::vector<Eigen::Vector3d> out(8,Eigen::Vector3d::Zero());
    auto choose=[](int n,int k){double v=1.;for(int j=1;j<=k;++j)v=v*(n-j+1)/j;return v;};
    for(int k=0;k<8;++k)for(int i=0;i<=k;++i)
        out[k]+=c.col(7-i)*std::pow(duration,i)*choose(k,i)/choose(7,i);
    return out;
}
template<class Curve>
bool certifyKinematics(const Curve& trajectory,double velocity,double acceleration,double jerk){
    if(trajectory.getPieceNum()==0)return false;
    int budget=10000;
    for(int i=0;i<trajectory.getPieceNum();++i){
        const double t=trajectory[i].getDuration();if(!std::isfinite(t) || t<=0.)return false;
        auto c=positionControls(trajectory[i].getCoeffMat(),t);
        for(double limit:{velocity,acceleration,jerk}){
            const double scale=double(c.size()-1)/t;
            std::vector<Eigen::Vector3d> derivative(c.size()-1);
            for(size_t j=0;j<derivative.size();++j)derivative[j]=scale*(c[j+1]-c[j]);
            c.swap(derivative);if(!normHullInside(c,limit,budget))return false;
        }
    }
    return true;
}
template<class Planes>
bool hullInsidePlanes(const std::vector<Eigen::Vector3d>& c,const Planes& planes,int& budget,int depth=0){
    if(--budget<0 || c.empty() || !planes.allFinite())return false;
    auto inside=[&](const Eigen::Vector3d& p){
        return p.allFinite() && (planes.leftCols(3)*p+planes.col(3)).maxCoeff()<=1e-6;
    };
    bool all=true;for(const auto& p:c)all=all && inside(p);
    if(all)return true;
    if(!inside(c.front()) || !inside(c.back()) || depth>=18)return false;
    auto work=c,left=c,right=c;const size_t n=c.size();
    for(size_t level=1;level<n;++level){
        for(size_t i=0;i<n-level;++i)work[i]=.5*(work[i]+work[i+1]);
        left[level]=work[0];right[n-1-level]=work[n-1-level];
    }
    return hullInsidePlanes(left,planes,budget,depth+1) && hullInsidePlanes(right,planes,budget,depth+1);
}
}
