"""Exercise the 1.17 patch against pinned upstream source and message schemas.

The C++ harness substitutes uORB/transport; this is not a PX4 firmware build.
"""
import importlib.util
import re
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT/'test/fixtures/px4_v1_17_0'
spec = importlib.util.spec_from_file_location('prepare_mavlink_117', ROOT/'scripts/prepare_mavlink.py')
patch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(patch)


def source(tmp_path):
    root = tmp_path/'px4'
    shutil.copytree(FIXTURE, root)
    return root


def test_117_patch_is_idempotent_and_dry_run_does_not_write(tmp_path):
    root = source(tmp_path)
    original = (root/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp').read_bytes()
    assert len(patch.prepare(root, True, 'v1.17.0')) == 3
    assert (root/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp').read_bytes() == original
    assert len(patch.prepare(root, baseline='v1.17.0')) == 3
    assert patch.prepare(root, baseline='v1.17.0') == []
    sync = (root/'src/modules/mavlink/mavlink_timesync.cpp').read_text()
    assert '_mavlink.get_channel(), now / 1000' in sync
    assert '_mavlink->get_channel()' not in sync
    ekf = (root/'src/modules/ekf2/EKF2.cpp').read_text()
    section = ekf.split('void EKF2::AdvertiseTopics()', 1)[1].split('// verbose logging', 1)[0]
    assert patch.ADVERTISE_117 in section
    assert section.index('ZAV_SINGLE_LINK_BARO_INSTANCE') < section.index('if (_multi_mode)')


def test_117_bad_schema_or_sync_anchor_prevents_partial_edits(tmp_path):
    root = source(tmp_path)
    stream = root/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp'
    before = stream.read_bytes()
    sync = root/'src/modules/mavlink/mavlink_timesync.cpp'
    sync.write_text(sync.read_text().replace('_timesync.update(now, tsync.tc1, tsync.ts1);', 'changed();'))
    with pytest.raises(ValueError):
        patch.prepare(root, baseline='v1.17.0')
    assert stream.read_bytes() == before
    aid = root/'msg/EstimatorAidSource1d.msg'
    aid.write_text(aid.read_text()+'\nbool fusion_enabled\n')
    with pytest.raises(ValueError, match='schema'):
        patch.prepare(root, baseline='v1.17.0')
    assert stream.read_bytes() == before


def test_117_multi_ekf_baro_topic_indices_survive_reverse_first_publish(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ unavailable')
    root = source(tmp_path)
    patch.prepare(root, baseline='v1.17.0')
    ekf = (root/'src/modules/ekf2/EKF2.cpp').read_text()
    section = ekf.split('void EKF2::AdvertiseTopics()\n{', 1)[1].split('\t// verbose logging', 1)[0]
    pubs = sorted(set(re.findall(r'(_\w+_pub)\.advertise\(\)', section)))
    declarations = '\n'.join(f'Pub {p}{{"{p}"}};' for p in pubs)
    cpp = tmp_path/'instance.cpp'
    cpp.write_text('''
#include <cassert>
#include <map>
#include <string>
#define CONFIG_EKF2_BAROMETER 1
std::map<std::string,int> next_index;
struct Pub {
 std::string topic;int instance=-1;
 void advertise(){if(instance<0)instance=next_index[topic]++;}
};
struct EKF2 {
 bool _multi_mode=true;
''' + declarations + '\n void AdvertiseTopics(){\n'+section+'\n}\n};\n' + '''
int main(){
 EKF2 first;first.AdvertiseTopics();
 EKF2 second;second.AdvertiseTopics();
 // With fusion OFF, real aid publication may first run in reverse task order.
 second._estimator_aid_src_baro_hgt_pub.advertise();
 first._estimator_aid_src_baro_hgt_pub.advertise();
 assert(first._estimator_aid_src_baro_hgt_pub.instance==first._estimator_status_pub.instance);
 assert(second._estimator_aid_src_baro_hgt_pub.instance==second._estimator_status_pub.instance);
 assert(first._estimator_aid_src_baro_hgt_pub.instance==0);
 assert(second._estimator_aid_src_baro_hgt_pub.instance==1);
}
''')
    binary = tmp_path/'instance'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
def test_117_cpp_diagnostic_semantics(tmp_path):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ unavailable')
    root = source(tmp_path)
    patch.prepare(root, baseline='v1.17.0')
    # Generate field declarations directly from the actual 1.17 message schemas.
    def fields(path):
        result = []
        types = {'uint64':'uint64_t', 'uint32':'uint32_t', 'uint8':'uint8_t',
                 'float32':'float', 'bool':'bool'}
        for line in path.read_text().splitlines():
            parts = line.split('#')[0].split()
            if parts:
                result.append(f'{types[parts[0]]} {parts[1]}{{}};')
        return '\n'.join(result)
    declarations = ('struct estimator_aid_source1d_s {'+fields(root/'msg/EstimatorAidSource1d.msg')+'};\n'
                    'struct estimator_status_flags_s {'+fields(root/'msg/EstimatorStatusFlags.msg')+'};\n')
    for name in ['estimator_selector_status', 'estimator_status', 'estimator_aid_source1d', 'estimator_status_flags']:
        p = tmp_path/f'uORB/topics/{name}.h'
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text('')
    p = tmp_path/'drivers/drv_hrt.h'
    p.parent.mkdir(parents=True)
    p.write_text('')
    header = root/'src/modules/mavlink/streams/ESTIMATOR_STATUS.hpp'
    cpp = tmp_path/'test.cpp'
    cpp.write_text(r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <type_traits>
#define ORB_ID(x) 0
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS 230
#define MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN 42
#define MAVLINK_NUM_NON_PAYLOAD_BYTES 12
#define MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN 18
#define PX4_ISFINITE(x) std::isfinite(x)
uint64_t now = 10000000;
uint64_t hrt_absolute_time() { return now; }
uint64_t hrt_elapsed_time(const uint64_t *t) { return now-*t; }
''' + declarations + r'''
struct estimator_selector_status_s { unsigned primary_instance{}; };
struct estimator_status_s {
 uint64_t timestamp{}; float vel_test_ratio{},pos_test_ratio{},hgt_test_ratio{},hdg_test_ratio{},hagl_test_ratio{},tas_test_ratio{},pos_horiz_accuracy{},pos_vert_accuracy{}; uint16_t solution_status_flags{};
};
struct mavlink_estimator_status_t {
 uint64_t time_usec{}; float vel_ratio{},pos_horiz_ratio{},pos_vert_ratio{},mag_ratio{},hagl_ratio{},tas_ratio{},pos_horiz_accuracy{},pos_vert_accuracy{}; uint16_t flags{};
};
estimator_aid_source1d_s aid{};
estimator_status_flags_s flags{};
bool change_ok=true;
unsigned primary=0;
namespace uORB { class Subscription {
 unsigned instance=0;
public:
 Subscription(int) {} bool advertised(){return true;} bool updated(){return true;}
 unsigned get_instance(){return instance;}
 bool ChangeInstance(unsigned i){if(change_ok)instance=i;return change_ok;}
 template<class T> bool copy(T *p){
  if constexpr(std::is_same_v<T,estimator_status_flags_s>)*p=flags;
  if constexpr(std::is_same_v<T,estimator_selector_status_s>)p->primary_instance=primary;
  return true;
 }
 template<class T> bool update(T *p){
  if constexpr(std::is_same_v<T,estimator_aid_source1d_s>)*p=aid;
  return true;
 }
}; }
class Mavlink { public: int get_channel(){return 0;} };
class MavlinkStream { protected: Mavlink *_mavlink; public:
 MavlinkStream(Mavlink*m):_mavlink(m){} virtual ~MavlinkStream()=default;
 virtual const char*get_name()const=0;virtual uint16_t get_id()=0;virtual unsigned get_size()=0;virtual bool send()=0;
};
int last_bits=-1, sends=0;
void mavlink_msg_named_value_float_send(int,uint32_t,const char*,float v){last_bits=int(v);++sends;}
void mavlink_msg_estimator_status_send_struct(int,const mavlink_estimator_status_t*){}
''' + f'\n#include "{header}"\n' + r'''
int main(){
 Mavlink m;auto*s=MavlinkStreamEstimatorStatus::new_instance(&m);
 aid.timestamp_sample=now-1000;aid.time_last_fuse=now-1000;aid.test_ratio=.1f;flags.timestamp=now;
 s->send();assert(last_bits==0); // No fusion: never pretend baro is active.
 flags.cs_baro_hgt=true;aid.fused=true;s->send();assert(last_bits==3);
 aid.innovation_rejected=true;s->send();assert(last_bits==5);
 aid.innovation_rejected=false;aid.time_last_fuse=now-600000;s->send();assert(last_bits==1);
 aid.time_last_fuse=now-1000;aid.test_ratio=NAN;s->send();assert(last_bits==1);
 aid.test_ratio=.1f;
 auto old=sends;aid.timestamp_sample=now-600000;s->send();assert(sends==old);
 aid.timestamp_sample=now-1000;flags.timestamp=now-2100000;s->send();assert(sends==old);
 flags.timestamp=now;primary=1;aid.estimator_instance=1;s->send();assert(last_bits==3&&sends==old+1);
 old=sends;primary=2;change_ok=false;s->send();assert(sends==old);
 delete s;
}
''')
    binary = tmp_path/'check'
    subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-I', str(tmp_path), str(cpp), '-o', str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
