"""Execute the actual FC watchdog policy using a host C++ compiler, without hardware."""
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT/'firmware_build/PX4-Autopilot/src/modules/commander/EvLossGuard.hpp'


def test_fc_ev_loss_guard_policy(tmp_path):
    compiler = shutil.which('g++')
    if not compiler or not HEADER.exists():
        pytest.skip('FC source and host C++ compiler required')
    source = tmp_path/'ev_guard.cpp'
    source.write_text(r'''
#include "EvLossGuard.hpp"
#include <cassert>
#include <iostream>
using Input = EvLossGuard::Input;
Input healthy() {
 Input in{}; in.now=1000000; in.enabled=true; in.armed=true;
 in.ev_fresh=true; in.ev_fusing=true; return in;
}
int main() {
 // Receipt AND sample timestamp checks reject zero, future and aged data.
 assert(!EvLossGuard::fresh(1000000,0,250000));
 assert(!EvLossGuard::fresh(1000000,1000001,250000));
 assert(EvLossGuard::fresh(1000000,750000,250000));
 assert(!EvLossGuard::fresh(1000000,749999,250000));
 // No activation while disarmed, disabled or before healthy fusion.
 for(int scenario=0; scenario<3; ++scenario) {
  EvLossGuard guard; auto in=healthy();
  if(scenario==0)in.armed=false;
  if(scenario==1)in.enabled=false;
  if(scenario==2)in.ev_fusing=false;
  guard.update(in);in.ev_fresh=false;in.now+=1000000;guard.update(in);
  assert(!guard.watching() && !guard.latched() && !guard.requestLand());
 }
 EvLossGuard guard; auto in=healthy();guard.update(in);assert(guard.watching());
 in.now+=260000;in.ev_fresh=false;guard.update(in);
 assert(guard.latched() && !guard.requestLand());const auto failure=guard.failureTime();
 // Parameter disable and data recovery cannot erase a failure.
 in.enabled=false;in.ev_fresh=true;in.now+=10000;guard.update(in);
 assert(guard.latched() && guard.failureTime()==failure);
 in.baro_good=true;guard.update(in);assert(!guard.requestLand());
 in.vertical_good=true;guard.update(in);
 assert(guard.baroConfirmed() && guard.requestLand() && !guard.timedOut());
 in.kill=true;guard.update(in);assert(!guard.requestLand());
 in.kill=false;in.pilot_takeover=true;guard.update(in);
 assert(guard.pilotTakeover() && !guard.requestLand());
 in.pilot_takeover=false;guard.update(in);assert(!guard.requestLand());
 in.armed=false;guard.update(in);assert(guard.latched() && !guard.requestLand());
 // Timeout requests native failsafe without misreporting barometer confirmation.
 EvLossGuard timeout;in=healthy();timeout.update(in);in.ev_fresh=false;in.now+=260000;timeout.update(in);
 const auto t=timeout.failureTime();in.now=t+2999999;timeout.update(in);assert(!timeout.requestLand());
 in.now=t+3000000;timeout.update(in);assert(timeout.requestLand() && timeout.timedOut() && !timeout.baroConfirmed());
 // No failure -> disarming removes session monitoring and requires new healthy fusion.
 EvLossGuard session;in=healthy();session.update(in);in.armed=false;session.update(in);
 in.armed=true;in.ev_fusing=false;in.ev_fresh=false;session.update(in);assert(!session.latched());
 std::cout << "FC EV guard scenarios passed\n";
}
''')
    binary = tmp_path/'ev_guard'
    subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror',
                    '-I',str(HEADER.parent),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
