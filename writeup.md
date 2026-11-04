## 1. Recap of the Four Tracking Steps

The project was implemented in four incremental steps: Extended Kalman Filtering, track management, multi-target data association, and camera-LiDAR sensor fusion.

### Step 1: Extended Kalman Filter

I implemented a 3D Extended Kalman Filter using a constant-velocity motion model. The state vector contains the 3D position and velocity of the tracked object

I implemented the system matrix `F`, process-noise covariance `Q`, state prediction, covariance prediction, measurement residual `gamma`, residual covariance `S`, Kalman gain, and measurement update.

The single-target test on Sequence 2, frames 150 to 200, achieved a mean RMSE of:

    0.32 m

This satisfied the required maximum mean RMSE of 0.35 m.

### Step 2: Track Management

I implemented automatic track initialization, scoring, state transitions, and deletion.

New tracks are initialized from unassigned LiDAR measurements. The initial position is transformed into vehicle coordinates, while velocity is initialized to zero.

Each new track starts with:

    state = initialized
    score = 1 / window

Successful measurement updates increase the score and allow tracks to progress through:

    initialized -> tentative -> confirmed

Unassigned tracks lose score when the object is inside the sensor field of view but is not detected. Tracks are deleted when their confidence becomes too low or their positional covariance becomes too large.

In the test scenario, the target was correctly initialized, confirmed, tracked, and eventually deleted after disappearing. The console successfully reported:

    deleting track no. 0

### Step 3: Multi-Target Data Association

I implemented Single Nearest Neighbor data association using the Mahalanobis distance

A chi-square gate rejects statistically implausible track-measurement combinations.

The valid pair with the smallest Mahalanobis distance is selected. The corresponding row and column are removed from the association matrix, ensuring that each measurement and each track can be used only once during a sensor-association cycle.

After implementing the association logic, the system successfully maintained multiple confirmed tracks simultaneously.

### Step 4: Camera-LiDAR Sensor Fusion

Finally, I implemented the nonlinear camera measurement model, including sensor field-of-view checking, coordinate transformation, nonlinear image projection, camera measurement generation, and camera measurement covariance.


Camera measurements were then associated with existing tracks and used for EKF updates after the LiDAR updates.

The final sensor-fusion experiment produced three confirmed tracks with the following mean RMSE values:

 Track  Mean RMSE 
 Track 0  0.17 m 
 Track 1  0.10 m 
 Track 14  0.13 m 

All three tracks achieved a mean RMSE below 0.25 m.

The most difficult part of the project was camera-LiDAR fusion. The tracked state exists in 3D vehicle coordinates, while camera observations exist in 2D image coordinates. Coordinate transformations, calibration, nonlinear projection, the camera Jacobian, field-of-view checking, covariance, gating, and association all have to remain consistent. Even a small error in one of these components can prevent valid camera measurements from being associated with the correct track.


## 2. Benefits of Camera-LiDAR Fusion Compared to LiDAR-Only Tracking

Camera and LiDAR provide complementary information.

LiDAR provides accurate metric range and 3D geometric information, while a camera provides dense observations in image space. Combining both sensors can reduce uncertainty, improve robustness, and provide additional observations when one sensor alone is insufficient.

The benefit was also visible in the project results. During the LiDAR-only track-management scenario, the supplied LiDAR detections contained a systematic lateral offset, which produced a relatively high tracking error. A standard Kalman Filter assumes approximately zero-mean measurement noise and therefore cannot fully compensate for a persistent systematic bias.

After introducing camera measurements, the final sensor-fusion experiment achieved:

    Track 0:  0.17 m
    Track 1:  0.10 m
    Track 14: 0.13 m

The final results demonstrate that combining camera observations with LiDAR tracking can improve the accuracy of the state estimates.

Sensor fusion also provides redundancy. If one sensor temporarily produces a weak or missing observation, another sensor may still provide useful information for maintaining the object track.


## 3. Challenges for Sensor Fusion in Real-Life Scenarios

Real-world sensor fusion introduces several challenges.

### Sensor Calibration

Camera and LiDAR measurements exist in different coordinate systems. Accurate intrinsic and extrinsic calibration is essential. Small calibration errors can create significant projection and association errors, particularly for distant objects.

### Time Synchronization

Different sensors may operate at different frequencies or capture observations at slightly different times. For moving objects, timestamp differences can cause spatial disagreement between measurements.

### Measurement Noise and Bias

Every sensor has different noise characteristics. LiDAR provides accurate range information but becomes increasingly sparse with distance. Cameras provide dense image observations but do not directly measure metric depth.

The project also demonstrated the effect of systematic measurement bias. The supplied LiDAR detections contained a lateral offset, which increased the LiDAR-only tracking error.

### Occlusion and Missed Detections

Vehicles may become temporarily occluded. The tracking system must continue predicting their motion without immediately deleting valid tracks.

### False Positives

Object detectors can produce incorrect detections. Track states and track scores are therefore important because they prevent isolated detections from immediately becoming trusted confirmed tracks.

### Data Association

Crowded scenes make it difficult to determine which measurement belongs to which track. Incorrect associations can create identity switches, duplicate tracks, ghost tracks, or incorrect filter updates.

Mahalanobis-distance gating reduces the number of plausible associations, but Single Nearest Neighbor is still a greedy approach.

### Environmental Conditions

Cameras can be affected by low light, glare, shadows, rain, and fog. LiDAR can also be affected by weather, reflective surfaces, and sparse returns from distant objects.

Sensor fusion is valuable because different sensors often have different failure modes.


## 4. Future Improvements

Several improvements could further increase tracking accuracy and robustness.

### Better Parameter Tuning

The process noise, measurement noise, confirmation threshold, deletion threshold, and covariance limits could be optimized using a larger validation dataset. Measurement covariance could also be estimated from real detection errors.

### Explicit Bias Estimation

The systematic LiDAR offset observed in the project could be explicitly modeled. The tracker could estimate an additional bias state, or calibration could be improved offline before applying the EKF.

### More Advanced Data Association

Single Nearest Neighbor could be replaced by more sophisticated approaches such as:

- Global Nearest Neighbor (GNN)
- Joint Probabilistic Data Association (JPDA)
- Multiple Hypothesis Tracking (MHT)

These approaches could improve association performance in crowded or ambiguous scenes.

### More Realistic Motion Models

The current implementation assumes constant velocity. Real vehicles accelerate and turn, so future implementations could use constant-acceleration, coordinated-turn, bicycle, or Interacting Multiple Model approaches.

### Extended State Vector

The state vector could be extended to estimate additional object properties, including:

- yaw
- yaw rate
- width
- length
- height

This would allow the complete 3D bounding box to be filtered over time.

### Real Camera Object Detection

The project uses camera labels as camera measurements. A more realistic implementation would use detections produced by an actual camera neural network and associate those detections with the LiDAR tracks.

### Additional Sensors

A production system could combine multiple cameras, additional LiDAR sensors, and radar. Radar would be particularly useful because it directly measures relative velocity and performs well under several adverse weather conditions.

### Adaptive Measurement Covariance

Measurement uncertainty could be adjusted dynamically according to observation quality. Distant, partially occluded, or low-confidence detections could be assigned larger covariance, while clear nearby detections could receive smaller covariance.
