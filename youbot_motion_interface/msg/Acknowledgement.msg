uint8 DEFAULT = 0

#Base motion action types
uint8 BASE_ACTION_TYPE_MOVE_BASE_POSE = 1
uint8 BASE_ACTION_TYPE_MOVE_BASE_NAME = 2
uint8 BASE_ACTION_TYPE_DBC_POSE = 3

#Arm motion action types
uint8 ARM_ACTION_TYPE_MOVEIT_NAME = 1
uint8 ARM_ACTION_TYPE_MOVEIT_POSE = 2
uint8 ARM_ACTION_TYPE_MOVEIT_JOINTS = 3
uint8 ARM_ACTION_TYPE_CARTESIAN_POSE = 4

#Status types
uint8 REQUEST_REJECTED = 0
uint8 REQUEST_ACCEPTED = 1

#Error types
uint8 NO_ERROR = 0
uint8 DIFFERENT_ACTION_IS_RUNNING_ERROR = 1
uint8 INVALID_POSE_NAME_ERROR = 2

string caller_id
uint8 base_action_type
string base_action_name
uint8 arm_action_type
string arm_action_name
uint8 status_type
string status
uint8 error_type
string error_msg

