# Sensor Fusion and Object Tracking

This repository implements a multi-object tracking and sensor-fusion pipeline using LiDAR and camera measurements from the Waymo Open Dataset. The project progresses through four stages: Extended Kalman Filtering, track management, multi-target data association, and camera-LiDAR sensor fusion.

## Final Results

| Project Step | Result |
|---|---:|
| Step 1, single-target EKF | Mean RMSE: **0.32 m** |
| Step 2, track management | Track initialized, confirmed, and deleted correctly |
| Step 3, multi-target association | Multiple confirmed tracks maintained |
| Step 4, Track 0 | Mean RMSE: **0.17 m** |
| Step 4, Track 1 | Mean RMSE: **0.10 m** |
| Step 4, Track 14 | Mean RMSE: **0.13 m** |

The final fusion experiment therefore satisfies the target of keeping the principal tracks below 0.25 m mean RMSE.

## Project Structure

The main implementations are located in:

```text
student/filter.py
student/trackmanagement.py
student/association.py
student/measurements.py
```

Supporting detection, visualization, and orchestration code includes:

```text
student/objdet_detect.py
student/objdet_pcl.py
student/objdet_eval.py
misc/evaluation.py
loop_over_dataset.py
```

For the detailed discussion requested by the project template, see [WRITEUP.md](WRITEUP.md).

## Step 1: Extended Kalman Filter

The first stage implements a six-dimensional constant-velocity EKF in `student/filter.py`. The tracked state is:

```text
x = [px, py, pz, vx, vy, vz]^T
```

The implementation includes the system matrix `F`, process-noise covariance `Q`, prediction, EKF update, measurement residual `gamma`, and residual covariance `S`.

The single-target scenario used Sequence 2, frames 150-200. The measured mean RMSE was **0.32 m**, satisfying the required threshold of 0.35 m.

<p align="center">
  <img width="2020" height="695" alt="Step 1 tracking result" src="https://github.com/user-attachments/assets/336df3e2-8345-4697-82e2-c3e68aeca29e" />
</p>

<p align="center">
  <img width="1327" height="891" alt="Step 1 RMSE" src="https://github.com/user-attachments/assets/ca467ab1-61dd-4c0c-8524-790defb7e361" />
</p>

## Step 2: Track Management

Track management is implemented in `student/trackmanagement.py`. New tracks are initialized from unassigned LiDAR measurements, including sensor-to-vehicle coordinate transformation and initialization of state covariance. Each new track starts in the `initialized` state with a score of `1 / window`.

Successful associations increase the score, allowing tracks to progress through:

```text
initialized -> tentative -> confirmed
```

Unassigned tracks inside the active sensor field of view lose score. Tracks are deleted when confidence becomes too low or positional covariance becomes too large. In the Step 2 scenario, the console correctly reported `deleting track no. 0` after the object disappeared.

<p align="center">
  <img width="1432" height="539" alt="Step 2 track-management result" src="https://github.com/user-attachments/assets/d90ffe6c-62d5-40eb-8a74-f4c7065e724f" />
</p>

## Step 3: Multi-Target Data Association

Multi-target association is implemented in `student/association.py` using Single Nearest Neighbor association and Mahalanobis-distance gating.

For each track-measurement pair, the squared Mahalanobis distance is computed as:

```text
MHD = gamma^T S^-1 gamma
```

A chi-square gate rejects statistically implausible pairs. The valid pair with the lowest distance is selected, and its row and column are removed from the association matrix so that each track and each measurement can be used at most once during one sensor-association cycle.

The Sequence 1 experiment successfully maintained multiple confirmed tracks simultaneously.

<p align="center">
  <img width="2508" height="907" alt="Step 3 multi-target tracking" src="https://github.com/user-attachments/assets/712645bd-001a-4895-b42f-c4243a09468f" />
</p>

## Step 4: Camera-LiDAR Sensor Fusion

The final stage adds the nonlinear camera measurement model in `student/measurements.py`, including:

- sensor field-of-view checking;
- vehicle-to-camera coordinate transformation;
- nonlinear image projection `h(x)`;
- camera measurement covariance;
- camera measurement generation;
- camera EKF updates after LiDAR updates.

The final Sequence 1 evaluation produced three confirmed tracks:

| Track | Mean RMSE |
|---:|---:|
| 0 | **0.17 m** |
| 1 | **0.10 m** |
| 14 | **0.13 m** |

<p align="center">
  <img width="2549" height="850" alt="Step 4 sensor-fusion result" src="https://github.com/user-attachments/assets/bb0903f3-c576-4937-a6a3-9513e37c1bf6" />
</p>

<p align="center">
  <img width="2531" height="872" alt="Step 4 final RMSE" src="https://github.com/user-attachments/assets/091558b5-53b6-4182-9409-2842278191ad" />
</p>

## Setup

The final project builds on the Udacity Sensor Fusion mid-term project and uses the same Python project structure and dependencies. Use the Udacity workspace environment or an equivalent local environment with the repository dependencies installed.

The project requires:

1. The Waymo Open Dataset sequences expected by `loop_over_dataset.py`.
2. The pre-computed LiDAR results supplied for the final project in the `results/` directory.
3. The project Python dependencies, including NumPy, SciPy, Matplotlib, OpenCV, PyTorch, and the supplied Waymo reader utilities.

A typical repository layout is:

```text
project/
├── dataset/
├── results/
├── misc/
├── student/
├── tools/
├── loop_over_dataset.py
├── README.md
└── WRITEUP.md
```

## Running the Final Sensor-Fusion Scenario

Configure `loop_over_dataset.py` for Sequence 1 and use:

```python
show_only_frames = [0, 200]
configs_det = det.load_configs(model_name='fpn_resnet')
configs_det.lim_y = [-25, 25]
exec_detection = []
exec_tracking = ['perform_tracking']
exec_visualization = ['show_tracks']
```

Then run:

```bash
python loop_over_dataset.py
```

`exec_detection = []` loads the supplied pre-computed LiDAR data instead of recomputing detections.

## Tracking Movie

To generate the tracking movie, use:

```python
exec_visualization = [
    'show_tracks',
    'make_tracking_movie'
]
```

After processing the sequence, the generated movie is written as:

```text
my_tracking_results.avi
```

## Syntax Check


```bash
python -m py_compile student/filter.py student/trackmanagement.py student/association.py student/measurements.py student/objdet_detect.py student/objdet_pcl.py student/objdet_eval.py misc/evaluation.py loop_over_dataset.py
```

A successful syntax check produces no output.

## Project Write-Up

The detailed answers to the project questions, including the discussion of sensor-fusion benefits, real-world challenges, and future improvements, are available in [WRITEUP.md](WRITEUP.md).
