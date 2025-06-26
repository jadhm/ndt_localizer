# NdtStat.msg (ROS 2 style, CamelCase, std_msgs/Header)
# Custom message for NDT localizer statistics

std_msgs/Header header
float32 exe_time
int32 iteration
float32 score
float32 velocity
float32 acceleration
int32 use_predict_pose
