#Arm motion action types
uint8 DEFAULT = 0
uint8 ACTION_TYPE_MOVEIT_NAME = 1
uint8 ACTION_TYPE_MOVEIT_POSE = 2
uint8 ACTION_TYPE_MOVEIT_JOINTS = 3
uint8 ACTION_TYPE_MOVEIT_POSE = 4

uint8 action_type
bool preempt_current_action
geometry_msgs/PoseStamped goal_pose
string goal_pose_name

