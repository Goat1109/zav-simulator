#pragma once
#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <vector>

namespace rog_map {
// Offsets are ordered by distance to the containing cell CENTER, not the
// requested sub-voxel position. An out-of-radius candidate cannot end the
// search until the triangle-inequality lower bound excludes all later shells.
template<typename Position,typename Matches>
bool findCellWithinRadius(const Eigen::Vector3i& start_id,const Eigen::Vector3d& start,
                          double radius,double resolution,const std::vector<Eigen::Vector3i>& offsets,
                          Position position,Matches matches,Eigen::Vector3d& result) {
    const Eigen::Vector3d query=start; // Several map callers use in-place input/output.
    result.setConstant(std::numeric_limits<double>::quiet_NaN());
    if(!query.allFinite() || !std::isfinite(radius) || radius<0. || !std::isfinite(resolution) || resolution<=0.)return false;
    const double shell_radius=radius+(position(start_id)-query).norm();
    for(const auto& offset:offsets) {
        if(offset.squaredNorm()*resolution*resolution>shell_radius*shell_radius+1e-12)break;
        const Eigen::Vector3d point=position(start_id+offset);
        if((point-query).squaredNorm()>radius*radius+1e-12)continue;
        if(matches(point)){result=point;return true;}
    }
    return false;
}
}
