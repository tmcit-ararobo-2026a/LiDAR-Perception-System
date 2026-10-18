# Observed bottom geometry in header.frame_id. This is not an opening/aim point.
# Consumers must check valid and the measurement stamp; stale data is invalid.
std_msgs/Header header
builtin_interfaces/Time last_observation_stamp
uint32 track_id
bool valid
geometry_msgs/Point center
geometry_msgs/Vector3 normal
float64 radius
float64 plane_rmse
float64 boundary_residual
uint32 plane_points
uint32 current_points
uint32 side_points
float32 confidence
