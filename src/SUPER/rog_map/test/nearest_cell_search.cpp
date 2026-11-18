#include <rog_map/rog_map_core/nearest_cell_search.hpp>
#include <algorithm>
#include <iostream>
#include <stdexcept>

int main() {
    using Eigen::Vector3d;using Eigen::Vector3i;
    std::vector<Vector3i> neighbors;
    for(int x=-4;x<=4;++x)for(int y=-4;y<=4;++y)for(int z=-4;z<=4;++z)neighbors.emplace_back(x,y,z);
    std::stable_sort(neighbors.begin(),neighbors.end(),[](const auto& a,const auto& b){return a.squaredNorm()<b.squaredNorm();});
    const auto position=[](const Vector3i& id){return ((id.cast<double>().array()+.5)*.15).matrix().eval();};
    Vector3d result;
    // The negative-X cell is outside the radius but occurs before the free
    // positive-X cell in the same integer-distance shell. The request is near
    // its containing cell's positive face, as happens to moving click goals.
    const Vector3d request(.149,.075,.075);
    auto only_positive=[](const Vector3d& p){return p.x()>.2&&p.y()==.075&&p.z()==.075;};
    if(!rog_map::findCellWithinRadius(Vector3i::Zero(),request,.1,.15,neighbors,position,only_positive,result))
        throw std::runtime_error("Sub-voxel nearest free cell was missed after an out-of-radius neighbor");
    if((result-Vector3d(.225,.075,.075)).norm()>1e-10||(result-request).norm()>.1)
        throw std::runtime_error("Nearest free cell exceeds the requested snap radius");
    result=request;
    if(!rog_map::findCellWithinRadius(Vector3i::Zero(),result,.1,.15,neighbors,position,only_positive,result) ||
       (result-Vector3d(.225,.075,.075)).norm()>1e-10)
        throw std::runtime_error("In-place map query overwrote its input before searching");
    if(rog_map::findCellWithinRadius(Vector3i::Zero(),request,.05,.15,neighbors,position,only_positive,result)||result.allFinite())
        throw std::runtime_error("Failed query must remain a failure with an invalid output");
    int visits=0;
    if(rog_map::findCellWithinRadius(Vector3i::Zero(),request,.1,.15,neighbors,position,
       [&visits](const Vector3d&){++visits;return false;},result)||visits>10)
        throw std::runtime_error("Radius search must stop after the bounded nearby shells");
    std::cout<<"PASS: sub-voxel free-cell lookup, strict radius and bounded search\n";
}
