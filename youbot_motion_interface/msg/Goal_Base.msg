#Base motion action types
uint8 DEFAULT = 0
uint8 ACTION_TYPE_MOVE_BASE_POSE = 1
uint8 ACTION_TYPE_MOVE_BASE_NAME = 2
uint8 ACTION_TYPE_DBC_POSE = 3

uint8 action_type
bool preempt_current_action
geometry_msgs/PoseStamped goal_pose
string goal_pose_name

