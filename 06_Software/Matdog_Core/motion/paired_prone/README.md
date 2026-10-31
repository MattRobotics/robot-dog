# Two-pair prone compiler (offline only)

Original MATDOG host geometry study. Does not link Controller, ServoBus, authority or persistence writers. Canonical URDF and all 17 collision meshes are inputs. Left/right profiles are mirrored through URDF transforms, never equal encoder ticks.

Set `MATDOG_PAIRED_OUTPUT` to an absolute directory outside the repository. Run `python3 build_adapters.py --output "$MATDOG_PAIRED_OUTPUT"`, then `paired_acquisition.py`, `continuous_pair_path.py`, `walk_after_stand.py`. NumPy/SciPy and g++ are required. The costly baseline is already preserved; compare pinned inputs before rerunning. Geometry checks do not grant execution.

The baseline begins with a synthetic belly-edge/foot-contact pose at positive pitch about 5.55 degrees. Pitch briefly reaches about 10.24 degrees, derived from contact closure. Four HIP angles become zero in the URDF after PRE-ALIGN; installed mechanical neutrals remain unverified. This is a restricted geometric start, not a measured torque-off rest pose and not a proof of a fully horizontal acquisition.

Collision-free nonadjacent swept bounds are distinct from the 3mm clearance policy. Adjacent swept certification, force distribution, friction/sliding, loaded tracking, gait swept checking and dynamics remain gates. See the night-run report; this route is intentionally not installed in the actuator-producing runtime. Existing generic Cartesian acquisition remains subject to its independent measured evidence and cannot replay this pitched route.
