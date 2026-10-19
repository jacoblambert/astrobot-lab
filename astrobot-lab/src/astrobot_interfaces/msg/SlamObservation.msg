# Immutable first available estimator-local evidence, paired to its first graph
# revision. This is GLIM's preprocessed/deskewed cloud, NOT unfiltered raw LiDAR.
string session_id
uint64 localization_epoch
uint64 revision
SlamFramePose frame
sensor_msgs/PointCloud2 points
bool preprocessing_lossy
