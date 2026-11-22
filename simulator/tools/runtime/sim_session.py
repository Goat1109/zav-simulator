"""Small, validated rendezvous record for the single local simulator session.

The record contains configuration only. It lets navigation launched from a
second terminal use the same ROS domain, even when shell startup files differ.
"""
import argparse
import json
import os
from pathlib import Path
import tempfile
import time

WORKSPACE = Path(__file__).resolve().parents[3]
RUNNER = WORKSPACE / 'simulator/scripts/run_argo_sim.sh'


def session_path():
    return Path(f'/tmp/argo-sim-session-{os.getuid()}.json')


def read_session(path=None):
    path = Path(path) if path else session_path()
    try:
        if path.stat().st_uid != os.getuid():
            return None
        data = json.loads(path.read_text())
        pid = int(data['pid'])
        domain = int(data['domain'])
        if pid <= 1 or not 0 <= domain <= 232:
            return None
        proc = Path('/proc') / str(pid)
        if proc.stat().st_uid != os.getuid():
            return None
        argv = (proc / 'cmdline').read_bytes().split(b'\0')
        if os.fsencode(RUNNER) not in argv:
            return None
        if data['scene'] not in ('warehouse', 'high_speed', 'dense', 'mixed', 'dasan', 'frontier'):
            return None
        data['domain'] = domain
        data.setdefault('vehicle', 'argo_v1')
        if data['vehicle'] not in ('argo_v1', 'argo_v2'):
            return None
        return data
    except (OSError, ValueError, TypeError, KeyError):
        return None


def write_session(pid):
    data = {'pid': int(pid), 'domain': int(os.environ.get('ROS_DOMAIN_ID', '0')),
            'vehicle': os.environ.get('ARGO_VEHICLE', 'argo_v1'),
            'calibration_dir': os.environ.get('PEGASUS_FASTLIVO_CONFIG_DIR', ''),
            'scene': os.environ.get('ARGO_SUPER_SCENE') or 'warehouse',
            'transport': os.environ.get('ARGO_ROS_TRANSPORT', 'fastdds'),
            'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY', '0')}
    if os.environ.get('ARGO_SUPER_MISSION_FILE'):
        data['mission'] = str(Path(os.environ['ARGO_SUPER_MISSION_FILE']).expanduser().resolve())
    path = session_path()
    with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                     prefix=f'.argo-sim-session-{os.getuid()}-', delete=False) as f:
        json.dump(data, f)
        temporary = Path(f.name)
    temporary.replace(path)


def clear_session(pid):
    path = session_path()
    try:
        if path.stat().st_uid == os.getuid() and json.loads(path.read_text()).get('pid') == int(pid):
            path.unlink()
    except (OSError, ValueError, TypeError):
        pass


def wait_for_session(scenario, timeout=30., vehicle=None):
    deadline = time.monotonic() + timeout
    while True:
        session = read_session()
        if session:
            if vehicle and session['vehicle'] != vehicle:
                raise RuntimeError(f"Simulator aircraft is {session['vehicle']}, requested {vehicle}")
            if session['scene'] != scenario:
                raise RuntimeError(f"Simulator scene is {session['scene']}, requested {scenario}. "
                                   f"Restart with ./run_super_sim.sh {scenario}")
            return session
        if time.monotonic() >= deadline:
            raise RuntimeError(f'No running simulator session. Start ./run_super_sim.sh {scenario} first.')
        time.sleep(.2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('write', 'clear', 'check-vehicle'))
    parser.add_argument('--pid', type=int)
    args = parser.parse_args()
    if args.action == 'check-vehicle':
        session=read_session()
        requested=os.environ.get('ARGO_VEHICLE','argo_v1')
        if not session or session['vehicle'] != requested:
            parser.exit(2, 'A different aircraft is already running; stop its simulator before switching.\n')
    else:
        if args.pid is None: parser.error('--pid is required')
        (write_session if args.action == 'write' else clear_session)(args.pid)
