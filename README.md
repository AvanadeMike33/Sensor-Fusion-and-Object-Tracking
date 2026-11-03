# Sensor-Fusion-and-Object-Tracking

This project implements a complete multi-object tracking and sensor-fusion pipeline using real-world LiDAR and camera measurements from the Waymo Open Dataset.

# Step 1 - Extended Kalman Filter

The first step implements an Extended Kalman Filter capable of tracking a single vehicle using LiDAR measurements.

The EKF implementation is located in:

    student/filter.py

## Step 1 Result

The single-target scenario was evaluated on Sequence 2 between frames 150 and 200.

The resulting mean RMSE was:

    Mean RMSE: 0.32 m

The project requirement was:

    Mean RMSE <= 0.35 m

Therefore, the Extended Kalman Filter successfully passed the Step 1 requirement.

Suggested evidence:

<img width="2020" height="695" alt="image" src="https://github.com/user-attachments/assets/336df3e2-8345-4697-82e2-c3e68aeca29e" />

 <img width="1327" height="891" alt="image" src="https://github.com/user-attachments/assets/ca467ab1-61dd-4c0c-8524-790defb7e361" />
 


# Step 2 - Track Management

The second step implements automatic track initialization, scoring, state transitions, and deletion.

The implementation is located in:

    student/trackmanagement.py


## Dynamic Track Initialization

A new track is initialized whenever an unassigned LiDAR measurement is available.

The measurement is transformed from sensor coordinates into vehicle coordinates using the sensor transformation matrix.


The position is initialized directly from the transformed LiDAR measurement.

The initial velocity is set to zero because velocity cannot be directly determined from a single measurement.

The positional covariance is derived from the measurement covariance and transformed into vehicle coordinates.



# Track Score

The score describes the confidence that a track represents a real physical object.

After a successful measurement association:

    score = score + 1 / window

with the score limited to a maximum value of `1.0`.

For an unassigned track inside the relevant sensor field of view:

    score = score - 1 / window

A track outside the sensor field of view is not penalized because the absence of a measurement is expected in this situation.



# Track States

Tracks transition between three states:


The transition to `confirmed` occurs when the score reaches the configured `confirmed_threshold`.

This prevents isolated false measurements from immediately becoming valid confirmed tracks.

## Step Result

The Step 2 test uses Sequence 2 between frames 65 and 100.

The scenario contains a vehicle that appears, remains visible for a period of time, and then disappears.

The console successfully reported:

    deleting track no. 0

The RMSE plot contained a single continuous track without an intermediate track loss.

The RMSE in this scenario is intentionally higher because the supplied LiDAR detections contain a systematic lateral offset that cannot be fully compensated by a Kalman Filter assuming zero-mean measurement error.

Suggested evidence:

<img width="1432" height="539" alt="image" src="https://github.com/user-attachments/assets/d90ffe6c-62d5-40eb-8a74-f4c7065e724f" />

 
# Step 3 - Multi-Target Data Association

The third step extends the solution from single-object tracking to multi-object tracking.

The implementation is located in:

    student/association.py

The association algorithm uses Single Nearest Neighbor association combined with Mahalanobis-distance gating.


# Association Matrix

For every frame, an association matrix is created.

Rows correspond to existing tracks and columns correspond to incoming measurements.

For every possible track-measurement combination, the Mahalanobis distance is calculated.

Track-measurement combinations that do not pass the statistical gate are assigned:

    infinity

This ensures that implausible associations cannot be selected by the nearest-neighbor algorithm.



## Step 3 Result

The multi-target scenario was evaluated using Sequence 1 across frames 0 to 200.

After implementing the complete association logic, the system successfully tracked multiple vehicles simultaneously.

The final visualization showed multiple confirmed tracks instead of the single-track behavior produced by the initial starter implementation.

The RMSE visualization contained separate curves for multiple successfully tracked vehicles.

Suggested evidence:

    
  <img width="2508" height="907" alt="image" src="https://github.com/user-attachments/assets/712645bd-001a-4895-b42f-c4243a09468f" />


# Step 4 - Camera and LiDAR Sensor Fusion

The fourth and final step introduces camera measurements and completes the sensor-fusion system.

The implementation is primarily located in:

    student/measurements.py

The following components were implemented:

- Sensor field-of-view checking
- Nonlinear camera measurement model
- Camera measurement generation
- Camera measurement covariance
- Camera-LiDAR EKF updates

# Sensor Field of View

Before associating a measurement with a track, the predicted object position is transformed from vehicle coordinates into sensor coordinates.

The horizontal bearing of the object is then calculated.

A track is considered visible when:

1. The object is in front of the sensor.
2. The horizontal bearing lies inside the configured sensor field of view.

This check is particularly important for track management.

A missing camera measurement should reduce a track score only when the predicted object is actually visible to the camera.


# Final Sensor-Fusion Results

The final Sequence 1 evaluation produced three confirmed tracks:

- Track 0
- Track 1
- Track 14

The measured mean RMSE values were:

| Track | Mean RMSE |
|------:|----------:|
| 0 | 0.17 m |
| 1 | 0.10 m |
| 14 | 0.13 m |

All three confirmed tracks achieved a mean RMSE below:

    0.25 m

The two principal tracks therefore comfortably satisfy the final project accuracy requirement.

Suggested evidence:

     
 <img width="2549" height="850" alt="image" src="https://github.com/user-attachments/assets/bb0903f3-c576-4937-a6a3-9513e37c1bf6" />
 
 <img width="2531" height="872" alt="image" src="https://github.com/user-attachments/assets/091558b5-53b6-4182-9409-2842278191ad" />
 
# Final Performance Summary

| Project Step | Result |
|---|---|
| Extended Kalman Filter | Completed |
| Step 1 Mean RMSE | 0.32 m |
| Step 1 Requirement | <= 0.35 m |
| Track Initialization | Completed |
| Track State Management | Completed |
| Track Score Management | Completed |
| Track Deletion | Completed |
| Single Nearest Neighbor Association | Completed |
| Mahalanobis Distance | Completed |
| Chi-Square Gating | Completed |
| Multi-Target Tracking | Completed |
| Camera Field-of-View Check | Completed |
| Nonlinear Camera Model | Completed |
| Camera EKF Updates | Completed |
| Camera-LiDAR Sensor Fusion | Completed |
| Final Track 0 RMSE | 0.17 m |
| Final Track 1 RMSE | 0.10 m |
| Final Track 14 RMSE | 0.13 m |


# Running the Final Sensor-Fusion Scenario

The final evaluation uses Sequence 1.

Relevant configuration:

    show_only_frames = [0, 200]

    configs_det = det.load_configs(
        model_name='fpn_resnet'
    )

    configs_det.lim_y = [-25, 25]

    exec_detection = []

    exec_tracking = [
        'perform_tracking'
    ]

    exec_visualization = [
        'show_tracks'
    ]

The project uses the pre-computed LiDAR detections supplied for the final project so that the tracking results are evaluated using consistent detection inputs.

Run the tracking pipeline using:

    python loop_over_dataset.py



# Tracking Movie

A tracking movie can be generated by changing the visualization configuration to:

    exec_visualization = [
        'show_tracks',
        'make_tracking_movie'
    ]

The tracking loop stores each visualization frame and combines the frames into a video after processing the sequence.

The resulting file is:

    my_tracking_results.avi


The implementation begins with a three-dimensional constant-velocity Extended Kalman Filter and progressively introduces dynamic track management, probabilistic data association, statistical gating, and nonlinear camera measurements.

The final sensor-fusion system successfully maintains multiple confirmed vehicle tracks while combining complementary information from LiDAR and camera sensors.

LiDAR provides accurate three-dimensional position and object geometry, while camera measurements provide additional image-space observations that improve tracking accuracy and reduce the systematic error visible in the LiDAR-only scenario.

The final evaluation produced three confirmed tracks with mean RMSE values of 0.17 m, 0.10 m, and 0.13 m. These results demonstrate that the completed EKF, track-management, data-association, and camera-LiDAR fusion pipeline satisfies the final project tracking requirements.
