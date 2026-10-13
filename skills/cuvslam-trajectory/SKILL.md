---
name: cuvslam-trajectory
description: >-
  Replay recorded EuRoC/ASL, KITTI odometry, EDEX, or ROS bag data through NVIDIA
  cuVSLAM to export trajectory.txt, TUM poses, or KITTI poses, and optionally
  evaluate trajectories against ground truth with ATE and ARE. Also evaluate
  existing trajectories without replay, inspect replay inputs, and validate TUM
  files. Use cuvslam-onboard for first-time setup or live cameras, and
  cuvslam-troubleshoot for tracking failures or drift diagnosis.
license: NVIDIA Community License
allowed-tools: Read Glob Grep Bash Edit Write WebFetch
metadata:
  author: Zheng Wang <zhengwang@nvidia.com>
---

# cuVSLAM trajectory replay and evaluation

Use the bundled scripts instead of writing a one-off tracker. They handle dataset detection, the extra-camera-frame
case, released/source API differences, atomic output, and validation.

`<skill-dir>` is this installed skill directory; `<cuvslam-repo>` is a separate cuVSLAM checkout
containing `VERSION` and `tools/python_tools/`. Do not assume the skill is installed inside that
checkout. Inspection and TUM validation use Python's standard library and need no CUDA runtime.

## Route from the request

| Request | Workflow |
| --- | --- |
| Run cuVSLAM on a recording / export poses | Replay and validate using the replay workflow below. |
| Run cuVSLAM and evaluate / compare with GT / get metrics | Replay and validate, then follow the evaluation workflow. |
| Evaluate an existing trajectory against GT | Evaluation only; skip replay, GPU checks, and cuVSLAM runtime setup. |
| Validate an existing TUM file | Run replay step 4 directly. |
| Inspect recording metadata | Stop after replay step 2. |

Ground-truth files being present does not request evaluation. Follow user-specified metrics and conventions;
do not substitute a new replay for an existing trajectory supplied for evaluation.

## Defaults

Unless the user overrides them:

- output format: TUM, `timestamp tx ty tz qx qy qz qw`;
- filename: `trajectory.txt`;
- output directory: the supplied dataset directory, or the parent of a supplied dataset file;
- tracking mode: infer it from the data (`inertial` for EuRoC with IMU, `multicamera` for KITTI stereo, and from the
  rig/streams for EDEX).

Treat any user-specified format, filename, location, sequence, mode, calibration, or ROS topic mapping as
authoritative.

## Replay command options

For TUM output named `trajectory.txt` in the default output directory, omit output flags entirely. When the user
requests an override, use these exact `replay_dataset.py` options:

| Option | Meaning |
| --- | --- |
| `--output PATH` | Output filename or full path. Relative paths resolve in the default output directory, not the shell's working directory. |
| `--output-format FORMAT` | Either `tum` (default) or `kitti`. |

`--output-path` and `--output-file` are unsupported. The `output_path` key printed by `--inspect` is a JSON field,
not a command-line option. Before adding an option not shown here, check its exact spelling and accepted values:

```bash
python3 <skill-dir>/scripts/replay_dataset.py --help
```

## Replay workflow

1. Resolve the cuVSLAM repository and dataset paths. If the prompt does not identify one dataset and discovery finds
   multiple plausible inputs, ask which one to use.
2. Inspect without running:

   ```bash
   python3 <skill-dir>/scripts/replay_dataset.py <dataset> --repo <cuvslam-repo> --inspect
   ```

   Add `--sequence`, `--edex-config`, or `--dataset-format` when inspection reports ambiguity. Read
   [references/dataset-routing.md](references/dataset-routing.md) for format-specific flags, especially ROS bags.
3. Use a Python interpreter where both `cuvslam` and `cuvslam_tools` import. Prefer an existing compatible
   environment. Otherwise use the released-wheel bootstrap instead of starting a source build:

   ```bash
   python3 <skill-dir>/scripts/bootstrap_runtime.py --repo <cuvslam-repo>
   # Default: TUM trajectory.txt in the dataset directory, or beside a dataset file.
   <cuvslam-repo>/.venv-trajectory/bin/python \
       <skill-dir>/scripts/replay_dataset.py <dataset> --repo <cuvslam-repo>
   ```

   If the user requests a specific output location, pass it with `--output`:

   ```bash
   <cuvslam-repo>/.venv-trajectory/bin/python \
       <skill-dir>/scripts/replay_dataset.py <dataset> --repo <cuvslam-repo> \
       --output /requested/output/trajectory.txt --output-format tum
   ```

   The bootstrap selects the wheel by repository version, CUDA major, Python ABI, glibc, and architecture. It is
   safe to rerun after an interrupted install. If CUDA detection is ambiguous, inspect `/usr/local/cuda` and pass
   `--cuda-major 12|13`; do not select the driver-reported maximum CUDA version unless that runtime is installed. If
   no released wheel matches, explain the mismatch. Build from source only when necessary and compatible with the
   user's time/compute constraints; run it in the foreground and do not leave a polling wrapper as the deliverable.
4. The replay script validates TUM output automatically. For an existing or externally generated TUM file, run:

   ```bash
   python3 <skill-dir>/scripts/validate_tum.py <trajectory-path> [--expected-rows N]
   ```

5. Report success only after the replay command exits zero and validation prints `"valid": true`. State the absolute
   output path, output format, poses written versus frames, tracking losses, and any deliberately excluded unmatched
   inputs.

## Evaluation workflow

Read [references/evaluation.md](references/evaluation.md) before evaluating. Use the independent
`scripts/evaluate_trajectory.py` helper (NumPy; PyYAML only for sensor YAML). It needs neither a product checkout
nor cuVSLAM/CUDA. Inspect its `--help` for supported input formats and exact options.

1. Identify each trajectory's physical frame (camera, body, IMU, or rig), world convention, pose direction,
   quaternion ordering, timestamp units, and calibration source. Inspect the actual exporter and calibration,
   including `sensor.yaml`; do not infer frames from filenames or tracking mode alone.
2. Convert physical frames with the full available rigid extrinsic, including rotation and translation, in the
   correct direction. For known identical physical frames, use identity. Camera/body conversion acts on the
   right and does not establish a shared world origin or orientation.
3. **World registration is also required for evaluation when the world frames differ**, even with correct
   extrinsics. Apply a known world transform with `--gt-world-from-estimate-world world.json` when available.
   Otherwise use `--alignment initial-poses --alignment-window 1.0` after physical-frame conversion. Inspect
   the fit diagnostics and apply one fixed SE(3) transform on the left, with scale 1, to all evaluated poses.
   The helper defaults to `--alignment auto`: supplied world transform, no fit for a verified shared world,
   otherwise initial-pose registration. Use `--alignment none` for tracking accuracy only when the worlds
   already coincide. Preserve actual world labels; do not relabel them to bypass registration. Do not fit
   scale, repeatedly realign, or enlarge the fit window to reduce later drift.
4. Associate timestamps within the overlapping interval and report exclusions. If the required extrinsic cannot
   be found or derived, `--allow-missing-extrinsics` permits a provisional comparison only: world registration
   cannot generally recover missing sensor calibration. Do not omit available calibration. If the user explicitly
   requests an unaligned diagnostic between different worlds, use `--alignment none`, label the residuals
   provisional, and explain that they include arbitrary world offsets rather than isolated tracking error.
5. Follow requested metrics. Otherwise report translation ATE RMSE in meters, rotation-angle ARE RMSE in degrees,
   and normalized ATE RMSE = `100 * ATE_RMSE / GT traveled distance` in percent. Report ARE in degrees unless
   the user defines a percentage denominator. Segment drift in `%` and `degrees/m` is a separate metric.
   If a requested metric is unsupported by the helper, calculate it separately with its stated definition;
   do not relabel a default metric to satisfy the request.
6. Report numerical results only after a successful computation. Include frames, calibration source and transform
   direction, alignment method/window/rationale and fit diagnostics, timestamp association, evaluated/excluded
   pose counts, metric definitions and units, limitations, and absolute output paths. Preserve command/exit evidence.

## Invariants

- Do not claim that a background build or replay "will finish". Stay with the foreground command until it exits.
- Do not fabricate, copy, or substitute ground-truth poses for tracker output.
- Do not delete dataset rows, images, or calibration files to force synchronization. The replay script intersects
  EuRoC camera timestamps in memory and reports unmatched inputs.
- Do not silently invent ROS topics, a rig frame, or extrinsics. A ROS bag needs a matching extraction config.
- Do not launch Rerun for a trajectory-only request. The bundled path is headless.
- Preserve provider-native exit status and completion output when an evaluator may need execution provenance.
- If the user asks for KITTI output, pass `--output-format kitti`; otherwise keep the TUM default even for KITTI
  input.
