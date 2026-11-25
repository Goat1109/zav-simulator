#!/usr/bin/env bash
set -eo pipefail
package_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
real_workspace="$(cd -- "$package_root/../.." && pwd)"
config_file="${1:-$package_root/config/hardware.yaml}"
agent_bin="${MICRO_XRCE_AGENT:-$real_workspace/.local/agent/bin/MicroXRCEAgent}"
[ -x "$agent_bin" ] || { echo 'MicroXRCEAgent not found; run install_agent.sh or set MICRO_XRCE_AGENT.' >&2; exit 1; }
# Private Agent DDS libraries must not enter the ROS processes' library search path.
exec python3 - "$config_file" "$agent_bin" <<'PY'
import os,sys,yaml
from pathlib import Path
profile=yaml.safe_load(Path(sys.argv[1]).read_text())
if profile.get('transport')=='mavlink':
    raise SystemExit('The current one-USB-TTL profile uses MAVLink only. Do not run a DDS Agent.')
c=profile['agent']
command=[sys.argv[2]]
if c['transport']=='serial':
    if not Path(c['device']).exists():raise SystemExit('Set agent.device to the real USB-TTL/GPIO UART device first')
    command+=['serial','--dev',c['device'],'-b',str(c['baud'])]
elif c['transport']=='udp4':command+=['udp4','-p',str(c['udp_port'])]
else:raise SystemExit('Unsupported transport')
env=dict(os.environ,ROS_DOMAIN_ID=str(c['domain_id']))
prefix=Path(sys.argv[2]).resolve().parents[1]
libs=[prefix/'lib',prefix/'lib64',prefix/'lib/aarch64-linux-gnu',prefix/'lib/x86_64-linux-gnu']
env['LD_LIBRARY_PATH']=':'.join(str(p) for p in libs if p.is_dir())
os.execvpe(command[0],command,env)
PY
