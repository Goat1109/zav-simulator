"""Run the FC payload latch policy with synthetic inputs only; no ROS/FC I/O."""
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT/'firmware_build/PX4-Autopilot/src/lib/mixer_module/functions/RcAuxToggle.hpp'


def test_rc_aux_toggle_policy(tmp_path):
    compiler = shutil.which('g++')
    if not compiler or not HEADER.exists():
        pytest.skip('FC source and host C++ compiler required')
    source = tmp_path/'aux_toggle.cpp'
    source.write_text(r'''
#include "RcAuxToggle.hpp"
#include <cassert>
#include <limits>
#include <iostream>
using Time=uint64_t;
void hold(RcAuxToggle &x, Time &now, float value, bool low=false, int count=6) {
 for(int i=0;i<count;++i){now+=20000;x.update(now,now,value,true,low);}
}
void turnOn(RcAuxToggle &x, Time &now, bool low=false) {
 hold(x,now,low?1.f:-1.f,low);hold(x,now,low?-1.f:1.f,low);assert(x.on());
}
int main(){
 Time now=1000000;RcAuxToggle x;assert(!x.on());
 // Boot while pressed does not turn on, regardless of how long it is held.
 hold(x,now,1.f,false,100);assert(!x.on());
 hold(x,now,-1.f);assert(!x.on());hold(x,now,1.f);assert(x.on());
 hold(x,now,1.f,false,100);assert(x.on()); // no auto-repeat
 hold(x,now,-1.f);assert(x.on()); // release retains light
 hold(x,now,1.f);assert(!x.on()); // second press switches off
 // A short contact spike cannot turn on.
 hold(x,now,-1.f);hold(x,now,1.f,false,2);assert(!x.on());
 hold(x,now,-1.f);assert(!x.on());hold(x,now,1.f);assert(x.on());
 // Brief release bounce cannot re-arm a held button.
 hold(x,now,-1.f,false,2);hold(x,now,1.f);assert(x.on());
 // Invalid RC forces OFF and requires a new stable release.
 now+=20000;x.update(now,now,1.f,false);assert(!x.on());
 hold(x,now,1.f);assert(!x.on());turnOn(x,now);
 // A stopped input stream resets without needing a new message.
 now+=RcAuxToggle::TimeoutUs+1;x.checkTimeout(now);assert(!x.on());
 hold(x,now,1.f);assert(!x.on());turnOn(x,now);
 // Fresh recovery after a long gap also resets before processing the sample.
 now+=RcAuxToggle::TimeoutUs+1;x.update(now,now,1.f,true);assert(!x.on());
 hold(x,now,1.f);assert(!x.on());turnOn(x,now);
 // Nonfinite, out-of-range, zero/future/old timestamps are all invalid.
 for(int scenario=0;scenario<7;++scenario){
  RcAuxToggle z;turnOn(z,now);now+=20000;Time stamp=now;float value=1.f;
  if(scenario==0)value=std::numeric_limits<float>::quiet_NaN();
  if(scenario==1)value=std::numeric_limits<float>::infinity();
  if(scenario==2)value=2.f;
  if(scenario==3)stamp=0;
  if(scenario==4)stamp=now+1;
  if(scenario==5)stamp=now-RcAuxToggle::TimeoutUs-1;
  if(scenario==6)stamp=now-30000; // older than the last accepted sample
  z.update(now,stamp,value,true);assert(!z.on());
  hold(z,now,1.f);assert(!z.on());
 }
 // Re-publication of one raw RC sample cannot debounce a press.
 RcAuxToggle repeated;hold(repeated,now,-1.f);now+=20000;const auto raw=now;
 repeated.update(now,raw,1.f,true);
 for(int i=0;i<10;++i){now+=20000;repeated.update(now,raw,1.f,true);}
 assert(!repeated.on());
 // A center-returning button works too; it need not return all the way to -1.
 RcAuxToggle centered;hold(centered,now,0.f);hold(centered,now,1.f);assert(centered.on());
 // Hysteresis band cannot become a confirmed press.
 RcAuxToggle noise;hold(noise,now,-1.f);hold(noise,now,.35f);assert(!noise.on());
 hold(noise,now,1.f);assert(noise.on());
 // Active-low momentary buttons use the same behavior.
 RcAuxToggle inverted;hold(inverted,now,-1.f,true);assert(!inverted.on());
 turnOn(inverted,now,true);hold(inverted,now,1.f,true);assert(inverted.on());
 hold(inverted,now,-1.f,true);assert(!inverted.on());
 // A fresh instance does not retain a previous instance's ON state.
 RcAuxToggle rebooted;assert(!rebooted.on());
 std::cout<<"AUX1 latch scenarios passed\n";
}
''')
    binary = tmp_path/'aux_toggle'
    subprocess.run([compiler,'-std=c++14','-O2','-Wall','-Wextra','-Werror',
                    '-I',str(HEADER.parent),str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)


def test_actual_rc_function_provider_with_mocked_px4_io(tmp_path):
    """Compile the actual output provider, replacing only PX4 time/params/uORB."""
    compiler = shutil.which('g++')
    provider = HEADER.with_name('FunctionManualRC.hpp')
    if not compiler or not provider.exists():
        pytest.skip('FC source and host C++ compiler required')
    for folder in ('drivers','parameters','uORB/topics'):
        (tmp_path/folder).mkdir(parents=True,exist_ok=True)
    (tmp_path/'uORB/topics/manual_control_setpoint.h').write_text(r'''
#pragma once
#include <cstdint>
struct manual_control_setpoint_s {
 static constexpr uint8_t SOURCE_RC=1;
 uint64_t timestamp_sample{};bool valid{};uint8_t data_source{};
 float roll{},pitch{},throttle{},yaw{},flaps{},aux1{},aux2{},aux3{},aux4{},aux5{},aux6{};
};
''')
    (tmp_path/'drivers/drv_hrt.h').write_text(r'''
#pragma once
#include <cstdint>
extern uint64_t fake_now;
inline uint64_t hrt_absolute_time(){return fake_now;}
''')
    (tmp_path/'parameters/param.h').write_text(r'''
#pragma once
#include <cstdint>
#include <cstring>
#include <cassert>
extern int32_t fake_mode;
inline int param_find(const char *name){assert(std::strcmp(name,"RC_AUX1_TOG")==0);return 0;}
inline int param_get(int,void *out){*static_cast<int32_t*>(out)=fake_mode;return 0;}
''')
    (tmp_path/'fake_provider.hpp').write_text(r'''
#pragma once
#include <cmath>
#include <uORB/topics/manual_control_setpoint.h>
extern manual_control_setpoint_s fake_sample;
extern bool fake_updated;
enum class OutputFunction {RC_Roll,RC_Pitch,RC_Throttle,RC_Yaw,RC_Flaps,RC_AUX1,RC_AUX2,RC_AUX3,RC_AUX4,RC_AUX5,RC_AUXMax};
namespace uORB {
struct Subscription {
 explicit Subscription(int){}
 bool update(manual_control_setpoint_s *out){if(!fake_updated)return false;*out=fake_sample;fake_updated=false;return true;}
};
}
#define ORB_ID(name) 0
class FunctionProviderBase {
public:struct Context{};virtual ~FunctionProviderBase()=default;
 virtual void update()=0;virtual float value(OutputFunction)=0;
};
''')
    (tmp_path/'ActualFunctionManualRC.hpp').write_text(provider.read_text().replace(
        '#include "FunctionProviderBase.hpp"','#include "fake_provider.hpp"'))
    source=tmp_path/'provider.cpp'
    source.write_text(r'''
#include "ActualFunctionManualRC.hpp"
#include <cassert>
#include <iostream>
uint64_t fake_now=1000000;int32_t fake_mode=0;
manual_control_setpoint_s fake_sample{};bool fake_updated=false;
using F=OutputFunction;
void publish(FunctionManualRC &p,float aux,bool valid=true,uint8_t source=1){
 fake_now+=20000;fake_sample={};fake_sample.timestamp_sample=fake_now;fake_sample.valid=valid;
 fake_sample.data_source=source;fake_sample.roll=.1f;fake_sample.pitch=-.2f;
 fake_sample.throttle=-.7f;fake_sample.yaw=.3f;fake_sample.flaps=.4f;
 fake_sample.aux1=aux;fake_sample.aux2=.2f;fake_sample.aux3=.3f;
 fake_sample.aux4=.4f;fake_sample.aux5=.5f;fake_sample.aux6=.6f;
 fake_updated=true;p.update();
}
void hold(FunctionManualRC &p,float x){for(int i=0;i<6;++i)publish(p,x);}
void others(FunctionManualRC &p){
 assert(p.value(F::RC_Roll)==.1f && p.value(F::RC_Pitch)==-.2f);
 assert(p.value(F::RC_Throttle)==-.7f && p.value(F::RC_Yaw)==.3f);
 assert(p.value(F::RC_Flaps)==.4f && p.value(F::RC_AUX2)==.2f && p.value(F::RC_AUX3)==.3f);
 assert(p.value(F::RC_AUX4)==.4f && p.value(F::RC_AUX5)==.5f && p.value(F::RC_AUXMax)==.6f);
}
int main(){
 // Default mode is exactly the old proportional passthrough.
 FunctionManualRC normal;publish(normal,.37f);assert(normal.value(F::RC_AUX1)==.37f);others(normal);
 publish(normal,-1.f);assert(normal.value(F::RC_AUX1)==-1.f);
 publish(normal,1.f,false);assert(std::isnan(normal.value(F::RC_AUX1)));
 fake_mode=1;FunctionManualRC p;publish(p,-1.f);assert(p.value(F::RC_AUX1)==-1.f);
 hold(p,1.f);assert(p.value(F::RC_AUX1)==-1.f); // no confirmed release since boot
 hold(p,-1.f);hold(p,1.f);assert(p.value(F::RC_AUX1)==1.f);others(p);
 hold(p,-1.f);assert(p.value(F::RC_AUX1)==1.f);others(p);
 hold(p,1.f);assert(p.value(F::RC_AUX1)==-1.f);
 hold(p,-1.f);hold(p,1.f);assert(p.value(F::RC_AUX1)==1.f);
 // Timeout is checked without any new publication.
 fake_now+=RcAuxToggle::TimeoutUs+1;p.update();assert(p.value(F::RC_AUX1)==-1.f);
 hold(p,1.f);assert(p.value(F::RC_AUX1)==-1.f);
 hold(p,-1.f);hold(p,1.f);assert(p.value(F::RC_AUX1)==1.f);
 publish(p,1.f,true,2);assert(p.value(F::RC_AUX1)==-1.f); // MAVLink cannot toggle
 others(p); // source restriction applies only to the payload latch
 hold(p,-1.f);hold(p,1.f);assert(p.value(F::RC_AUX1)==1.f);
 publish(p,1.f,false);assert(p.value(F::RC_AUX1)==-1.f);
 assert(std::isnan(p.value(F::RC_Throttle))); // original invalid-RC behavior retained
 fake_mode=2;FunctionManualRC inverted;hold(inverted,1.f);hold(inverted,-1.f);
 assert(inverted.value(F::RC_AUX1)==1.f);hold(inverted,1.f);assert(inverted.value(F::RC_AUX1)==1.f);
 // Constructor-only option changes cannot alter a live provider mid-session.
 fake_mode=0;publish(inverted,1.f);assert(inverted.value(F::RC_AUX1)==1.f);
 hold(inverted,-1.f);assert(inverted.value(F::RC_AUX1)==-1.f);
 std::cout<<"Actual RC output provider integration scenarios passed\n";
}
''')
    binary=tmp_path/'provider'
    subprocess.run([compiler,'-std=c++14','-O2','-Wall','-Wextra','-Werror',
                    '-Wno-unused-parameter','-I',str(tmp_path),'-I',str(HEADER.parent),
                    str(source),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
