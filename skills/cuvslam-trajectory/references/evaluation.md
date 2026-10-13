# Ground-truth evaluation

Use this workflow only when evaluation or metrics are requested. Existing estimates can be evaluated on a CPU
without replay, a GPU, cuVSLAM, or a product checkout. The helper uses NumPy; reading sensor YAML also uses PyYAML.
Use an existing environment with those packages or install only the missing evaluation dependencies in a venv.
Do not bootstrap the replay runtime for evaluation alone.

## Inputs and conventions

Inspect the source/exporter and calibration before choosing flags. The helper accepts:

| Format flag | Pose rows | Time | Rotation |
| --- | --- | --- | --- |
| `tum` | `timestamp tx ty tz qx qy qz qw` | seconds | active quaternion, xyzw |
| `euroc` | 17-column state ground-truth CSV: time, position, quaternion, velocity, gyro bias, accel bias | nanoseconds | active quaternion, wxyz |
| `kitti` | 12 row-major values of a 3x4 pose, plus `--estimate-timestamps` / `--gt-timestamps` | separate `times.txt`, seconds | rotation matrix |

All inputs must mean `T_world_from_sensor`: sensor coordinates map into world coordinates. Positions and calibration
translations must be in meters; rotations must be proper right-handed rotations. Convert inverse poses, other units,
other quaternion orders, or handedness explicitly before invoking the helper and record the conversion. Comments
begin with `#`. Non-finite data, repeated/out-of-order timestamps, invalid quaternion norms, scaled/reflected matrices,
and trajectories with fewer than two poses are rejected. Near-unit quaternions (norm error at most 0.001) are normalized.
Timestamps are parsed as decimal numbers then rounded to integer nanoseconds, avoiding epoch-float precision loss.
Static trajectories are valid evaluation inputs, even though replay validation normally expects motion.

KITTI files have no timestamps. Use the actual times corresponding to their rows; after dropped tracking frames,
the recording's complete `times.txt` is not a valid association unless it was filtered with the same frame indices.
Do not invent uniformly spaced timestamps.

## Physical-frame conversion first

`T_A_B` maps points in B to A. With estimate poses `T_We_E` and GT poses `T_Wg_G`, choose G as the evaluation sensor:

```text
converted estimate = T_We_E @ T_E_G
compare converted estimate with GT directly (--alignment none)
```

Sensor/body conversion multiplies on the **right**, using the full rotation and lever-arm translation.
When calibration is available, this skill uses that conversion alone: **do not apply world alignment**,
including an initial-pose reset or a separate global fit. Known identical physical frames also use no alignment.
World alignment is reserved for the explicit missing-extrinsics fallback described below.

This is an evaluation policy, not a claim that extrinsics establish the world registration. If the estimate and
GT have different worlds, the unaligned residual includes that world offset. Preserve their actual world labels
and report the helper's provisional result and limitation; do not present it as isolated tracking accuracy or
silently add an alignment to obtain smaller errors.

For two calibration files whose `T_BS` matrices both refer to the same physical reference B:

```text
T_E_G = inverse(T_B_E) @ T_B_G
```

Pass those files as `--estimate-sensor-yaml` and `--gt-sensor-yaml`. The helper reads their `T_BS` matrices and records
paths and SHA-256 hashes. Verify their common reference yourself: the field name alone does not establish it.
Alternatively, save a 4x4 row-major JSON matrix (flat 16 values or 4 rows) and pass `--estimate-from-gt transform.json`.
Record how this matrix was derived and the original calibration sources alongside the evaluation. Identity is
implicit only when `--estimate-frame` and `--gt-frame` name the same physical frame with the same axes.

The bundled EuRoC replay sets its rig to **cam0**, including in inertial mode; it exports `world_from_rig`.
For standard EuRoC state GT, inspect `state_groundtruth_estimate0/sensor.yaml` and the state CSV to establish the
body/IMU convention. With the standard calibration and identity body-to-IMU transform, comparing cam0 estimates
with IMU GT requires `inverse(T_B_cam0) @ T_B_imu0`. A supplied recording may use different conventions.
The replay helper selects a complete `sensor_recalibrated.yaml` family when available. Its recalibrated matrices
are already relative to cam0; do not combine them with body-relative default matrices as if both used the same B.
Establish the physical relationship to GT from the actual selected calibration and exporter.

Example, **after verifying** that GT is imu0, the default YAML files share the same reference B, and these describe
the replay's physical frames:

```bash
python3 <skill-dir>/scripts/evaluate_trajectory.py \
    /data/rec_05/trajectory.txt /data/rec_05/mav0/state_groundtruth_estimate0/data.csv \
    --gt-format euroc --estimate-frame cam0 --gt-frame imu0 \
    --estimate-world 'cuVSLAM local world, right-handed' \
    --gt-world 'EuRoC ground-truth world, right-handed' \
    --estimate-sensor-yaml /data/rec_05/mav0/cam0/sensor.yaml \
    --gt-sensor-yaml /data/rec_05/mav0/imu0/sensor.yaml \
    --alignment none \
    --output-dir /data/rec_05/evaluation
```

Only if the required extrinsics cannot be found or derived, explain the missing relationship and use
`--allow-missing-extrinsics --alignment initial-poses --alignment-window 1.0` for a provisional initial-window
comparison. This explicitly assumes identity for the missing conversion and emits a `provisional` status and
limitation. The helper rejects world alignment when calibrated conversion or a known same-sensor relationship
is available. Do not omit available calibration to enable this fallback. One world transform cannot generally replace a camera/body transform: a
lever arm moves with sensor rotation, and a missing orientation extrinsic also produces motion-dependent error.
Do not present these numbers as isolated tracking accuracy or claim the fit recovered the calibration. In an
already calibrated dataset, failure to inspect available extrinsics is not a justification for this fallback.

## Timestamp association and coverage

Default tolerance is 0.01 seconds. An optional `--time-offset SECONDS` is **added to estimate timestamps**;
use a known synchronization correction, not an automatically optimized offset. Clocks must share an epoch and
rate. The helper restricts both streams to their common timestamp interval, forms every candidate pair within
tolerance, sorts by `(absolute time difference, estimate index, GT index)`, and greedily takes unused pairs.
Matches are one-to-one; ties are deterministic; crossing matches are rejected. No interpolation or extrapolation
occurs. Inspect the measured maximum time difference and choose a justified tolerance for the sampling rates and
motion. If interpolation is requested, use and document a separate interpolation step including its gap policy.

Report total/evaluated/excluded pose counts and separate exclusions outside overlap from unmatched poses inside it.
A short common interval does not establish full recording coverage. Alignment poses remain in the reported errors.
`errors.csv` records the matched row indices (zero-based, excluding comments), corrected estimate and GT times,
time differences, translation errors, and rotation errors.

## Alignment policy and scale

`--alignment none` is the default. With available extrinsics, apply only the physical-frame conversion, associate
poses, and compute the metrics. Do not first reset the trajectories, fit their first poses/window, fit the entire
trajectory, or optimize scale. With the same verified physical frame, use the identity conversion and no alignment.

Keep the estimate's and GT's actual world-frame labels. For `none`, matching labels declare a verified shared
world. Different labels are accepted so the requested extrinsics-only residuals can be computed, but the report
is `provisional` and warns that those values include an unresolved world offset. Do not change labels merely to
make the report `ok`. Sensor extrinsics alone do not determine the world-to-world transform.

World alignment is available **only with `--allow-missing-extrinsics`**, after establishing that the required
physical-frame transform is unavailable. Both fallback modes keep scale exactly 1:

| `--alignment` | Missing-extrinsics fallback method |
| --- | --- |
| `initial-poses` | Mean of `R_gt @ R_est.T`, projected onto SO(3) by SVD; translation is `mean(p_gt - R_align @ p_est)`. Uses orientations even at rest or on a straight line. |
| `initial-positions` | Rigid Kabsch fit on initial positions; requires at least 2 independent centered position directions. |

Use a declared initial window, normally `--alignment-window 1.0` seconds from the first matched estimate timestamp,
inclusive. At least 3 pairs are required. Inspect fit residuals and conditioning, and record the actual fit count,
interval, matrix, and rationale. Apply one fixed transform to all evaluated poses; do not continually realign or
increase the window to improve the reported error. A fallback fit remains provisional because it does not recover
a missing camera/body extrinsic or isolate tracking error. Similarity/Sim(3) scale fitting is not supported.

## Metrics and output

Follow the user's requested definitions. Compute these errors after calibrated conversion alone, or after the
explicit missing-extrinsics fallback. With unresolved world frames or extrinsics, label them provisional residuals:

| Output | Definition |
| --- | --- |
| `ate_rmse_m` | `sqrt(mean(||p_est - p_gt||²))`, meters |
| `are_rmse_deg` | `sqrt(mean(angle(R_gt.T @ R_est)²))`, degrees; principal angle in [0, 180] |
| `normalized_ate_rmse_percent` | `100 * ate_rmse_m / gt_traveled_distance_m` |

The distance denominator sums **all consecutive native GT positions between the first and last matched GT times**,
in the GT physical frame. It is not endpoint displacement or path length of only the matched subset. It may include
GT movement during gaps in estimated poses; disclose exclusions and gaps when interpreting this metric. A zero
GT distance produces JSON `null` and a warning, not infinity or zero percent. Inspect large GT gaps/noise, which
can distort the distance denominator. ARE percent has no default: ask for or clearly establish a denominator when
requested. Translation segment drift `%` and rotational segment drift `degrees/m` must be named and computed
separately with segment lengths, sampling, and aggregation documented. They are not these absolute RMSE metrics.

`--output-dir` is required. A successful command writes `evaluation.json` and `errors.csv` there and prints the
JSON report; existing output files are never overwritten. The report records definitions, units, paths and hashes,
frame conventions and conversion, timestamp policy and counts, alignment diagnostics, warnings, and invocation.
Report the actual numerical results, limitations, and absolute paths; preserve exit status and computation logs.
Do not claim that structural validation or a successful metric calculation proves cuVSLAM execution provenance.
