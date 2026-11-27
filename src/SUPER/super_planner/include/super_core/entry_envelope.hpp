#pragma once
#include "super_core/planning_envelope.hpp"

namespace super_planner {
// One convex, heading-aligned approach corridor in world coordinates. Its
// vertical extent is always clipped by the independent aircraft height limits.
inline PlanningEnvelope entryEnvelope(const Eigen::Vector3d& start,const Eigen::Vector3d& goal,
                                      double floor,double ceiling) {
    PlanningEnvelope e;
    e.enabled=e.ready=true;
    e.inset=0.;
    e.world_floor=floor;
    e.world_ceiling=ceiling;
    const Eigen::Vector2d d=(goal-start).head<2>();
    const double yaw=std::atan2(d.y(),d.x());
    e.rotation=Eigen::AngleAxisd(-yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    e.translation=-e.rotation*start;
    const Eigen::Vector3d end=e.profile(goal), zero=Eigen::Vector3d::Zero();
    const Eigen::Vector3d pad=Eigen::Vector3d::Constant(.35);
    e.lower.push_back(zero.cwiseMin(end)-pad);
    e.upper.push_back(zero.cwiseMax(end)+pad);
    return e;
}
}
