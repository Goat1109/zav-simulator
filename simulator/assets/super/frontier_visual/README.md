# Frontier photographic terrain

This is the default photographic Frontier terrain selected by
`./run_super_sim.sh frontier`. The original voxel asset remains available,
and the mission and common planner configuration are unchanged. The separate
`view.sh` command opens only a map viewer; it starts no flight stack.

Open the map in Isaac Sim:

```bash
./simulator/assets/super/frontier_visual/view.sh
```

The viewer opens a recorded corridor viewpoint. Buttons select a lower corridor,
an upper corridor, a stair landing or the whole building. The camera is free to
orbit; there is no vehicle chase camera. A floor cutaway is available to inspect
interior surfaces. The timeline remains stopped.

The primary asset is `frontier_textured.usdc` with relative texture references in
`textures/`. Keep `photo_surface.mdl` and the texture directory next to the USD
when copying the asset. The MDL material renders both sides of thin measured
surfaces; a standard USD preview material is also included.

Geometry comes from the saved FAST-LIVO2 complete world scan fragments of the
Frontier bag, fused at 5 cm and reconstructed as a continuous screened-Poisson
surface. Unsupported extrapolation is cropped against measured points. Camera
poses use the same rigid map alignment and saved camera/IMU extrinsics. Lens
undistortion, visibility checks, distance and view angle determine photograph
selection. UV atlases contain only actual recorded camera images. Surfaces
without a usable photograph retain the original measured point colors.

Photographic appearance includes the recorded lighting; materials use baked
emission to avoid adding the same illumination a second time. This is not a
new calibration of physical surface reflectance. The static mesh includes
collision attributes for import, and separate flight records are listed below. A full-route arrival was observed before the landing fix. After correcting
the local floor winding and adding a thin landing pad, native landing/disarming
passed two consecutive local tests; the complete route was not rerun.

Unobserved areas and thin railings may retain gaps or texture seams; this is
a reconstruction of the available measurements, not a complete architectural model.

The final runtime geometry and photographic atlases are distributed in split
compressed archives under `../frontier_package/`. The launcher and `view.sh`
automatically restore missing assets; Git LFS is not required. Raw bag files,
point/mesh/camera intermediate arrays and local reconstruction/test logs are
excluded from Git. The final asset opens without those intermediate files.

## Fly the existing Frontier mission in this terrain

The separate flight composition uses the photographic surface for both rendering,
LiDAR and physical collision. It includes only the same original start pad from
the voxel asset; the old voxel building is not loaded. Mission XYZ/yaw and
`common.yaml` remain unchanged. Collision uses the same triangles in spatial tiles with shared indexed
vertices, preserving local adjacency across photograph/height partitions;
visual meshes are excluded from duplicate collision. The static mesh clearance preflight sampled the
500.53956 m route every 2.5 cm and found a minimum centre-to-surface distance of
0.5593 m. This static distance alone does not establish collision-free flight.

```bash
./run_super_sim.sh frontier
# In another terminal, after Ready:
./run_super.sh --scenario frontier --execute
```

The launcher selects this photographic composition by default. An explicit
`ZAV_SUPER_TERRAIN_USD` override still takes precedence; select
`$PWD/simulator/assets/super/frontier.usdc` to use the original voxel terrain.
The flight composition must stay alongside the photographic USD, and it also
references `frontier_collision.usdc` for collision and `../frontier.usdc`
for its original start pad.

Flight records for this surface are under `log/frontier-20261119/textured-flight/`.
Trial01 and trial03 reached all 203 vertices with zero navigation impacts and
penetrations. Both failed to obtain native landed/disarmed confirmation after
contact with the top floor. Trial02 (one large collider) was aborted after early
stair contact. These are pre-repair results, not complete navigation-plus-landing successes. No mission or common planner YAML was changed.
Read-only local screening is in `landing-site-review.json`: current XY has full
support over a 0.45 m radius and 1.75 cm height range, but all first-hit floor
triangles in that footprint have downward winding. Adjacent candidates share
this orientation issue. Subsequent isolated tests reproduced the problem, confirmed that reversing
the local floor winding permits native landing, and validated two landings with
a 1 m square, 2 cm thick pad. The default flight composition contains this repair
and pad. See `landing-pad.json`, `landing-verification.json` and
`docs/frontier-landing.md` for the tested scope.
