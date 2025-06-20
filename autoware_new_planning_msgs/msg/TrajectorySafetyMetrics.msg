# Comprehensive trajectory safety evaluation metrics

# Header with timestamp
std_msgs/Header header

# Trajectory identification
string trajectory_id
uint32 trajectory_sequence_number

# Time To Collision (TTC) metrics
float64 min_ttc_seconds              # Minimum TTC across all objects and trajectory points
float64 mean_ttc_seconds             # Average TTC when objects are detected
float64[] ttc_per_object             # TTC for each detected object (parallel to object_ids)
string[] object_ids                  # IDs of objects used in TTC calculation
uint32 critical_ttc_violations       # Number of points where TTC < threshold
float64 ttc_threshold_seconds        # TTC threshold used for violation detection

# Traffic rule compliance
bool speed_limit_violated            # Whether speed limit was exceeded
float64 max_speed_violation_mps      # Maximum speed over the limit (0 if no violation)
float64 speed_limit_mps              # Speed limit used for evaluation
uint32 speed_violation_duration_ms   # Duration of speed limit violations

bool lane_keeping_violated           # Whether vehicle left designated lane
float64 max_lateral_deviation_m      # Maximum lateral deviation from lane center
uint32 lane_violation_count          # Number of lane boundary crossings
float64 lane_width_m                 # Lane width used for evaluation

bool red_light_violated              # Whether trajectory violates red light
bool stop_sign_violated              # Whether trajectory violates stop sign
bool yield_sign_violated             # Whether trajectory violates yield sign
uint32 traffic_signal_violations     # Total traffic signal violations

# Collision risk assessment
bool potential_collision_detected    # Whether collision risk was identified
float64 collision_probability        # Estimated collision probability [0.0-1.0]
float64 minimum_safe_distance_m      # Minimum distance to nearest object
string[] high_risk_object_ids        # Objects with high collision risk

# Trajectory smoothness (comfort metrics)
float64 max_lateral_acceleration_mps2    # Maximum lateral acceleration
float64 max_longitudinal_acceleration_mps2  # Maximum longitudinal acceleration
float64 max_jerk_mps3                    # Maximum jerk (rate of acceleration change)
float64 max_angular_velocity_rps         # Maximum angular velocity

# Evaluation metadata
float64 evaluation_duration_seconds     # Time taken to compute these metrics
bool evaluation_successful              # Whether evaluation completed successfully
string error_message                   # Error description if evaluation failed
uint32 total_trajectory_points          # Number of points in evaluated trajectory
uint32 evaluated_time_horizon_ms        # Time horizon used for predictions