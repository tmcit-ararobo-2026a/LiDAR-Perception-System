uint8 NONE=0
uint8 DIRECT=1
uint8 SUPPORT_INFERRED=2
uint8 PREDICTED=3

std_msgs/Header header
builtin_interfaces/Time last_observation_stamp
uint32 track_id
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[36] state_covariance
uint8 observation_mode
bool valid
float32 confidence
uint32 support_points
float32 fit_residual
