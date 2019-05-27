#Base motion action types
uint8 DEFAULT = 0
uint8 BASE_ACTION_TYPE_LOC_NAME = 1
uint8 BASE_ACTION_TYPE_DBC_POSE = 2

uint8 base_action_type
bool stop_current_action
geometry_msgs/PoseStamped base_goal_pose
string base_goal_pose_name

