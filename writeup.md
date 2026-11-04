# Writeup: Track 3D Objects Over Time

## 1. Recap of the Four Tracking Steps

The final project was completed incrementally through four stages: Extended Kalman Filtering, track management, multi-target data association, and camera-LiDAR sensor fusion.

### Step 1: Extended Kalman Filter

I implemented the Extended Kalman Filter in `student/filter.py`. The tracked state is:

```text
x = [px, py, pz, vx, vy, vz]^T
```

A three-dimensional constant-velocity process model predicts position and velocity. I implemented the system matrix `F`, process-noise covariance `Q`, state and covariance prediction, measurement residual, residual covariance, Kalman gain, and EKF update.

The residual is evaluated using the sensor measurement function:

```text
gamma = z - h(x)
```

rather than assuming only a linear LiDAR model. This allows the same EKF implementation to support the nonlinear camera measurement model used in Step 4.

For the single-target Sequence 2 scenario between frames 150 and 200, the final mean RMSE was **0.32 m**, satisfying the required maximum of 0.35 m.

<p align="center">
  <img width="1327" height="891" alt="Step 1 RMSE" src="https://github.com/user-attachments/assets/ca467ab1-61dd-4c0c-8524-790defb7e361" />
</p>

### Step 2: Track Management

I implemented track initialization, scoring, state transitions, and deletion in `student/trackmanagement.py`.

An unassigned LiDAR measurement initializes the position of a new track after transformation into vehicle coordinates. Velocity is initialized to zero because one measurement alone does not provide velocity. Position uncertainty is derived from the measurement covariance, while the initial velocity covariance uses the configured uncertainty parameters.

A new track begins with:

```text
state = initialized
score = 1 / window
```

Successful measurement updates increase the score. This allows progression through:

```text
initialized -> tentative -> confirmed
```

An unassigned track loses score only if the track lies inside the relevant sensor field of view. Confirmed tracks are deleted when their score becomes too low, while initialized and tentative tracks use more conservative deletion logic. Tracks are also removed if their positional covariance becomes excessive.

In the Step 2 scenario, the target was automatically initialized, tracked, confirmed, and later removed. The console successfully reported:

```text
deleting track no. 0
```

The LiDAR-only RMSE was intentionally higher in this scenario because the supplied detections contain a systematic lateral offset. A standard Kalman Filter cannot fully eliminate a persistent non-zero-mean measurement bias.

<p align="center">
  <img width="1432" height="539" alt="Step 2 track-management result" src="https://github.com/user-attachments/assets/d90ffe6c-62d5-40eb-8a74-f4c7065e724f" />
</p>

### Step 3: Multi-Target Data Association

I implemented Single Nearest Neighbor data association and gating in `student/association.py`.

For every possible track-measurement pair, the association matrix stores the squared Mahalanobis distance:

```text
MHD = gamma^T S^-1 gamma
```

where:

```text
gamma = z - h(x)
S = H P H^T + R
```

A chi-square gate rejects statistically implausible pairs by assigning them infinite association cost. The valid pair with the lowest Mahalanobis distance is selected. Its row and column are then removed from the matrix, ensuring that one track and one measurement can each be used at most once during one association cycle.

After implementing the complete association logic, Sequence 1 successfully produced simultaneous confirmed tracks for multiple vehicles.

<p align="center">
  <img width="2508" height="907" alt="Step 3 multi-target tracking" src="https://github.com/user-attachments/assets/712645bd-001a-4895-b42f-c4243a09468f" />
</p>

### Step 4: Camera-LiDAR Sensor Fusion

The final stage introduced the nonlinear camera measurement model in `student/measurements.py`.

I implemented sensor field-of-view checking, transformation from vehicle coordinates to camera coordinates, nonlinear projection into image coordinates, camera measurement generation, and camera measurement covariance.

The camera projection is:

```text
i = c_i - f_i * y / x
j = c_j - f_j * z / x
```

The EKF uses the corresponding camera Jacobian. The processing order is prediction, LiDAR association and update, camera association and update, and finally track management.

The final Sequence 1 sensor-fusion run produced three confirmed tracks with the following mean RMSE values:

| Track | Mean RMSE |
|---:|---:|
| 0 | **0.17 m** |
| 1 | **0.10 m** |
| 14 | **0.13 m** |

All three values are below 0.25 m.

<p align="center">
  <img width="2549" height="850" alt="Step 4 sensor-fusion result" src="https://github.com/user-attachments/assets/bb0903f3-c576-4937-a6a3-9513e37c1bf6" />
</p>

<p align="center">
  <img width="2531" height="872" alt="Step 4 final RMSE" src="https://github.com/user-attachments/assets/091558b5-53b6-4182-9409-2842278191ad" />
</p>

### Most Difficult Part

The most difficult part was integrating the nonlinear camera model with the already working LiDAR tracker. Camera observations are expressed in image coordinates while the track state exists in 3D vehicle coordinates, so calibration, coordinate transformations, field-of-view checks, the projection function, Jacobian, covariance, and data association all have to remain mathematically consistent.

The incremental four-step structure was valuable because each subsystem could be verified independently before enabling the next one.

## 2. Benefits of Camera-LiDAR Fusion

Camera and LiDAR provide complementary information. LiDAR directly measures three-dimensional structure and range, while a camera provides dense image-plane observations. Combining both modalities can reduce uncertainty and improve robustness when one sensor has weak or systematically biased measurements.

The project provided a concrete example. In the LiDAR-only track-management scenario, the supplied LiDAR detections contained a systematic lateral offset and the resulting RMSE was relatively high. The Kalman Filter assumes approximately zero-mean measurement noise, so it cannot completely correct a persistent measurement bias by itself.

After camera fusion was enabled, the final confirmed tracks achieved mean RMSE values of **0.17 m**, **0.10 m**, and **0.13 m**. The final experiment therefore demonstrates the practical value of adding a complementary camera observation to the LiDAR tracking pipeline.

Sensor fusion also adds redundancy. If one modality temporarily provides a weak observation, another sensor can help maintain a reliable state estimate, provided the measurements are correctly calibrated, synchronized, associated, and weighted.

## 3. Real-Life Sensor Fusion Challenges

A production sensor-fusion system has to handle significantly more uncertainty than the controlled project scenario.

### Calibration

Camera and LiDAR operate in different coordinate systems. Extrinsic calibration errors directly affect projection and association. Small angular errors can become large spatial errors for distant objects.

### Time Synchronization

Sensors may operate at different frequencies and may not capture the scene at exactly the same instant. For moving vehicles, even a small timestamp mismatch can cause disagreement between measurements.

### Measurement Noise and Bias

Different sensors have different uncertainty characteristics. LiDAR range is accurate but becomes spatially sparse with distance. Camera measurements are dense but do not directly provide metric depth. Noise covariances must represent these differences accurately.

The project also demonstrated systematic bias directly: the supplied LiDAR detections showed a lateral offset during the Step 2 experiment. This is especially challenging because a standard Kalman Filter assumes approximately zero-mean measurement noise.

### Occlusion and Missed Detections

Vehicles can be partially or completely hidden. During a temporary occlusion, the tracker must continue predicting motion without immediately deleting a valid object.

### False Positives and Ghost Tracks

Object detectors can produce incorrect measurements. The initialized and tentative track states help prevent isolated detections from immediately becoming confirmed tracks, while score reduction and covariance limits remove implausible tracks.

### Data Association

Crowded scenes create ambiguous track-measurement assignments. Incorrect matches can produce identity switches, duplicated objects, or ghost tracks. Mahalanobis gating reduces the candidate set, but Single Nearest Neighbor remains a greedy strategy.

### Environmental Conditions

Cameras can be affected by low light, glare, shadows, rain, and fog. LiDAR can also degrade in adverse weather or on problematic reflective surfaces. Multi-modal fusion is useful because different sensing technologies often have different failure modes.

## 4. Future Improvements

### Improved Parameter Tuning

The process noise, measurement noise, confirmation threshold, deletion threshold, and covariance limits could be optimized using a larger validation dataset. LiDAR and camera measurement covariances could also be estimated empirically from real detection errors.

### Bias Estimation

The systematic LiDAR offset observed in the project could be explicitly modeled. One possibility is augmenting the state with a bias term or estimating calibration bias offline and compensating for it before the EKF update.

### More Advanced Data Association

Single Nearest Neighbor is efficient but greedy. Potential alternatives include:

- Global Nearest Neighbor (GNN)
- Joint Probabilistic Data Association (JPDA)
- Multiple Hypothesis Tracking (MHT)

These methods can improve robustness in crowded and ambiguous scenes.

### Better Motion Models

The current constant-velocity model does not explicitly describe vehicle kinematics. Future implementations could use constant acceleration, coordinated-turn, bicycle, or Interacting Multiple Model approaches to improve prediction during acceleration and turning.

### Extended State Vector

Yaw, yaw rate, width, length, and height could be incorporated into the filter state instead of updating object dimensions separately from LiDAR measurements.

### Real Camera Detector

The project uses camera labels as camera measurements. A more realistic system would use detections from an actual camera neural network and associate the resulting image detections with the LiDAR tracks.

### Additional Sensors

Multiple cameras, additional LiDAR units, and radar could be fused. Radar would be particularly valuable because it directly measures relative velocity and remains useful under several adverse weather conditions.

### Adaptive Measurement Covariance

Measurement uncertainty could depend on observation quality. For example, distant, partially occluded, or low-confidence detections could receive larger covariance than clear nearby measurements.

The project successfully progressed from a single-target LiDAR EKF to a complete multi-target camera-LiDAR fusion system. The final experiment maintained three confirmed tracks with mean RMSE values of **0.17 m**, **0.10 m**, and **0.13 m**. The results show how EKF prediction and correction, track management, probabilistic association, and complementary sensor observations can be combined into a practical vehicle-tracking pipeline.
