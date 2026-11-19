# Frontier runtime package

These ordinary Git files contain the final FAST-LIVO2 Frontier map, actual
camera texture atlases, collision winding repair, landing pad and original
500.54 m mission. Chunks are at most 64 MiB. Git LFS is not required.

`./run_super_sim.sh frontier` and the photographic map viewer restore missing
assets automatically using Python standard-library code. Raw bags, reconstruction
intermediates and flight logs are excluded.

Manual restore: `python3 simulator/tools/super/restore_frontier_assets.py`.
Verify installed assets: add `--verify`. Existing files are preserved by default;
`--force` replaces them, including local mission edits, with this packaged version.
SHA-256 checks cover each chunk, the archive and every restored file.
