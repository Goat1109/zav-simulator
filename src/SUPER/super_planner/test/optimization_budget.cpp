#include <utils/optimization/lbfgs.h>
#include <super_core/initial_plan_policy.hpp>
#include <traj_opt/backup_initial_guess.hpp>
#include <traj_opt/config.hpp>
#include <stdexcept>
#include <thread>
#include <limits>

using Solver=math_utils::lbfgs;
static void require(bool test,const char *message){if(!test)throw std::runtime_error(message);}
struct Work { int calls{0}; bool slow_line_search{false}; };
static double cost(void *data,const Eigen::VectorXd&x,Eigen::VectorXd&g){
    auto &work=*static_cast<Work*>(data);++work.calls;
    if(work.slow_line_search && work.calls>1)std::this_thread::sleep_for(std::chrono::milliseconds(8));
    g=2.*x;return x.squaredNorm();
}
int main(){
    using Clock=std::chrono::steady_clock;
    Solver::lbfgs_parameter_t options;options.max_iterations=50;
    Eigen::VectorXd x=Eigen::VectorXd::Constant(2,2.);double f=0.;Work work;
    options.deadline=Clock::now()-std::chrono::milliseconds(1);
    require(Solver::lbfgs_optimize(x,f,cost,nullptr,nullptr,&work,options)==Solver::LBFGSERR_TIME_LIMIT,
            "An expired solve was not rejected");
    require(work.calls==0,"Expired solve evaluated the cost");
    work={0,true};options.deadline=Clock::now()+std::chrono::milliseconds(4);
    const auto begin=Clock::now();
    require(Solver::lbfgs_optimize(x,f,cost,nullptr,nullptr,&work,options)==Solver::LBFGSERR_TIME_LIMIT,
            "Line-search timeout was accepted as convergence");
    require(std::chrono::duration<double>(Clock::now()-begin).count()<.1,"Solver kept working after timeout");
    work={};x.setConstant(2.);options.deadline=Clock::now()+std::chrono::seconds(1);
    require(Solver::lbfgs_optimize(x,f,cost,nullptr,nullptr,&work,options)>=0 && x.norm()<1e-6,
            "A bounded feasible solve failed");
    require(super_planner::initialPlanFresh(10.05,10.,.1),"Fresh initial plan rejected");
    for(double stamp:{9.899,9.37,10.1,std::numeric_limits<double>::quiet_NaN()})
        require(!super_planner::initialPlanFresh(10.,stamp,.1),"Stale/future/nonfinite initial plan accepted");
    require(!super_planner::initialPlanFresh(10.,10.,-.1),"Invalid age budget accepted");
    const double hard=traj_opt::backupInitialDuration(.1899400795,2.019104,1.,.2983,6.,10.,5.);
    require(hard>=2.019104/6. && hard>.8,"Infeasible short stop seed was retained");
    require(!std::isfinite(traj_opt::backupInitialDuration(.1,1.,0.,1.,0.,10.,5.)),"Invalid dynamics seed accepted");
}
