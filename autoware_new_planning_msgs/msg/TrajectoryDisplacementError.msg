# Trajectory displacement error evaluation results

# Header with timestamp
std_msgs/Header header

# Trajectory identification
string trajectory_id
uint32 trajectory_sequence_number

# Ground truth trajectory information
uint32 ground_truth_points_count
uint32 candidate_points_count
uint32 evaluated_points_count

# Displacement error metrics
float64 average_displacement_error  # ADE: average Euclidean distance between all corresponding points
float64 final_displacement_error    # FDE: Euclidean distance between final points
float64 max_displacement_error      # Maximum displacement error across all points
float64 min_displacement_error      # Minimum displacement error across all points

# Additional statistics
float64 displacement_error_std      # Standard deviation of displacement errors
float64[] point_wise_errors         # Individual displacement error for each point

# Evaluation metadata
float64 evaluation_duration_seconds # Time taken to compute these metrics
bool evaluation_successful          # Whether evaluation completed successfully
string error_message               # Error description if evaluation failed