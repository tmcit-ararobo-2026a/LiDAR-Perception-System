# Bottom geometry in header.frame_id. This is not an opening/aim point.
# /bottom is confirmed geometry (DIRECT or SHAPE_FITTED);
# /bottom_track also includes tracked/inferred poses.
# Consumers must check valid and the measurement stamp; stale data is invalid.
std_msgs/Header header
builtin_interfaces/Time last_observation_stamp
uint32 track_id
bool valid
# DIRECT recognizes a dense bottom plane and side walls. PLANE_TRACKED uses current plane
# returns around a learned mount. BODY_INFERRED uses the fresh body pose only.
uint8 NONE=0
uint8 DIRECT=1
uint8 PLANE_TRACKED=2
uint8 BODY_INFERRED=3
# SHAPE_FITTED uses current frustum side evidence and a supported/learned
# bottom plane. A side-only update measures XY while retaining plane height.
uint8 SHAPE_FITTED=4
uint8 observation_mode
geometry_msgs/Point center
geometry_msgs/Vector3 normal
float64 radius
float64 plane_rmse
float64 boundary_residual
uint32 plane_points
uint32 current_points
uint32 side_points
float32 confidence
