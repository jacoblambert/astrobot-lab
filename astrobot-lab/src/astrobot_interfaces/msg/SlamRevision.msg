uint8 ODOM=1
uint8 SUBMAP=2
uint8 GLOBAL=3
std_msgs/Header header
string session_id
uint64 localization_epoch
uint64 revision
uint64 parent_revision
uint8 kind
# Complete snapshot of all frame poses known to this adapter, not all raw input scans.
bool full_snapshot
SlamFramePose[] frames
