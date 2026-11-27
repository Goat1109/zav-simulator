#!/usr/bin/env python3
"""Golden comparison with the pinned simulator PolylineFollower source snapshot."""
import importlib.util,math,random,subprocess,sys
from pathlib import Path
root=Path(__file__).resolve().parents[3]
ref=root/'analysis/epic_continuous_fix_20261125/sim_follower_reference.py'
spec=importlib.util.spec_from_file_location('sim_follower',ref);sim=importlib.util.module_from_spec(spec);spec.loader.exec_module(sim)
binary=root/'build/zav_super/path_follower_probe'
rng=random.Random(117)
paths=[[(0,0,1),(1,0,1),(2,0,1),(8,0,1)],[(0,0,1),(2,0,1),(0,0,1)],
       [(0,0,1),(2,2,1),(0,2,1),(2,0,1)],[(0,0,1),(2,0,1),(2,0,3),(4,0,3)],
       [(0,0,1),(0,0,1),(1,0,1),(1,1,2),(0,1,2),(0,0,1)]]
for _ in range(15):
    p=[(0.,0.,1.)]
    for _ in range(10):p.append(tuple(a+rng.uniform(-2,2) for a in p[-1]))
    paths.append(p)
checks=0
for points in paths:
    yaws=[rng.uniform(-math.pi,math.pi) for _ in points]
    follower=sim.PolylineFollower(points,4.,yaws,.15)
    positions=[]
    # Follow the sequential source polyline, including noise and occasional
    # backward movement. Dynamic preview shrinks and grows as in braking.
    for a,b in zip(points,points[1:]):
        for k in range(21):
            f=k/20;positions.append((tuple(x+(y-x)*f+rng.uniform(-.08,.08) for x,y in zip(a,b)),rng.uniform(.5,6.)))
    payload=f'{len(points)} {len(positions)} .15\n'
    payload+='\n'.join(' '.join(map(str,(*p,y))) for p,y in zip(points,yaws))+'\n'
    payload+='\n'.join(' '.join(map(str,(*p,look))) for p,look in positions)+'\n'
    output=subprocess.check_output([str(binary)],input=payload,text=True).splitlines()
    assert len(output)==len(positions)
    for line,(position,look) in zip(output,positions):
        follower.lookahead=look;target=follower.update(position)
        expected=[follower.progress,follower.target_arc,*target,follower.target_yaw]
        actual=list(map(float,line.split()))
        assert all(abs(a-b)<2e-6 for a,b in zip(expected,actual)),(points,position,expected,actual)
        checks+=1
print(f'PASS: {checks} ordered projection/preview steps match pinned simulator reference (20 routes, dynamic preview, 3D stairs/crossings/reversals/duplicates/yaw)')
