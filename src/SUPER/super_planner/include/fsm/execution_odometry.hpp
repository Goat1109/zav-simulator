#pragma once
#include <Eigen/Core>
#include <cmath>
#include <mutex>

namespace fsm {
// A coherent measurement snapshot owned by the independent execution reader.
// Preserve the message timestamp: dequeuing old odometry never makes it fresh.
class ExecutionOdometry {
public:
    struct State {
        Eigen::Vector3d position=Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity=Eigen::Vector3d::Zero();
        double stamp=-1.;
        bool received=false;
    };
    bool update(const Eigen::Vector3d& p,const Eigen::Vector3d& v,double stamp,double now) {
        if(!p.allFinite() || !v.allFinite() || !std::isfinite(stamp) || stamp<=0. ||
           !std::isfinite(now) || now-stamp<0. || now-stamp>.1)return false;
        std::lock_guard<std::mutex> lock(mutex_);
        if(state_.received && stamp<state_.stamp)return false;
        state_={p,v,stamp,true};return true;
    }
    State snapshot()const {
        std::lock_guard<std::mutex> lock(mutex_);return state_;
    }
private:
    mutable std::mutex mutex_;
    State state_;
};
}
