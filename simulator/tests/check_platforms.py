"""Check real ROS env selection and a stubbed native Isaac launcher without touching the live simulator."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
for script in root.rglob('*.sh'):
    subprocess.run(['bash', '-n', str(script)], check=True)
for platform, expected in [('native', 'rmw_fastrtps_cpp'), ('wsl', 'rmw_cyclonedds_cpp')]:
    env = dict(os.environ, ARGO_ENV_FILE='/nonexistent-argo-test-config', ARGO_SIM_PLATFORM=platform, ARGO_ROS_TRANSPORT='auto', ROS_DOMAIN_ID='49')
    code = 'source "$1" && python3 -c \'import os,json; print(json.dumps({k:os.environ.get(k) for k in ["RMW_IMPLEMENTATION","ROS_DOMAIN_ID","CYCLONEDDS_URI"]}))\''
    output = subprocess.check_output(['bash', '-c', code, 'test', str(root / 'tools/rviz/ros_env.sh')], env=env, text=True)
    values = json.loads(output)
    assert values['RMW_IMPLEMENTATION'] == expected, values
    assert values['ROS_DOMAIN_ID'] == '49'
    assert bool(values['CYCLONEDDS_URI']) == (platform == 'wsl'), values
with tempfile.TemporaryDirectory(prefix='argo-platform-') as temp:
    temp = Path(temp)
    home, binaries, isaac = temp / 'home', temp / 'bin', temp / 'Isaac with spaces'
    for path in [home, binaries, isaac]: path.mkdir()
    (binaries / 'tmux').write_text('#!/bin/sh\nexit 0\n')
    (binaries / 'sleep').write_text('#!/bin/sh\nexit 0\n')
    # Mimic the correct isaacsim_env/bin/python case that previously used isaac_env by mistake.
    python = home / 'isaacsim_env/bin/python'
    python.parent.mkdir(parents=True)
    python.write_text('''#!/bin/bash
if [ "$1" = -c ]; then echo "$ISAAC_TEST_ROOT"; exit 0; fi
/usr/bin/python3 - "$@" <<'CHECK'
import os,sys,json
assert os.environ['ROS_DISTRO']=='humble'
assert os.environ['RMW_IMPLEMENTATION']=='rmw_fastrtps_cpp'
assert 'PYTHONPATH' not in os.environ
assert 'CYCLONEDDS_URI' not in os.environ
assert '/opt/ros/' not in os.environ['LD_LIBRARY_PATH']
assert os.environ['ISAAC_PATH']==os.environ['ISAAC_TEST_ROOT']
assert sys.argv[1].endswith('/simulator/argo_sim.py')
print('NATIVE_ISAAC_ENV_OK', flush=True)
CHECK
exit 7
''')
    for path in [python, binaries / 'tmux', binaries / 'sleep']: path.chmod(0o755)
    env = dict(os.environ, HOME=str(home), XDG_CONFIG_HOME=str(home / '.config'), ARGO_SIM_PLATFORM='native',
               ARGO_ENV_FILE=str(temp / 'nonexistent.env'), ARGO_VEHICLE='argo_v1',
               PEGASUS_FASTLIVO_CONFIG_DIR=str(root.parent / 'src/FAST-LIVO2/config'),
               PEGASUS_DIR=str(temp / 'Pegasus'), ARGO_WS_DIR=str(root.parent),
               ISAAC_TEST_ROOT=str(isaac), PYTHONPATH='/opt/ros/humble/lib/python3.10/site-packages',
               LD_LIBRARY_PATH='/opt/ros/humble/lib:/some/workspace/install/lib', CYCLONEDDS_URI='stale',
               PATH=str(binaries) + ':' + os.environ['PATH'])
    env.pop('ISAAC_PYTHON', None); env.pop('ISAAC_SIM_PATH', None)
    result = subprocess.run(['bash', str(root / 'scripts/run_isaac_sim.sh')], env=env, text=True, capture_output=True, timeout=20)
    assert result.returncode == 7, (result.returncode, result.stdout, result.stderr)
    assert 'NATIVE_ISAAC_ENV_OK' in result.stdout
print('PLATFORMS_OK: native/WSL DDS, domain preservation, native interpreter, ABI isolation, exit propagation')
