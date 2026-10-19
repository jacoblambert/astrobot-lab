# Identity is (session_id from SlamRevision, frame_id), never receipt time.
int64 frame_id
builtin_interfaces/Time measurement_stamp
# -1 means active/unassigned to a finalized submap.
int64 submap_id
geometry_msgs/Pose map_from_imu
geometry_msgs/Pose imu_from_lidar
# Pose revisions do not imply that the original within-scan deskew was recomputed.
bool deskew_recomputed
