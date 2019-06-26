# Arm motion action types
uint8 ACTION_TYPE_DEFAULT = 0
uint8 ACTION_TYPE_MOVEIT_POSE = 1
uint8 ACTION_TYPE_MOVEIT_NAME = 2
uint8 ACTION_TYPE_MOVEIT_JOINTS = 3
uint8 ACTION_TYPE_CVC_POSE = 4

uint8 action_type
bool preempt_current_action
geometry_msgs/PoseStamped target_pose
float32[5] target_joint_configuration
string target_pose_name
