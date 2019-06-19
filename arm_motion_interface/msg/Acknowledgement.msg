# Arm motion action types
uint8 DEFAULT = 0
uint8 ACTION_TYPE_MOVEIT_NAME = 1
uint8 ACTION_TYPE_MOVEIT_POSE = 2
uint8 ACTION_TYPE_MOVEIT_JOINTS = 3
uint8 ACTION_TYPE_CVC_POSE = 4

# Status types
uint8 REQUEST_REJECTED = 0
uint8 REQUEST_ACCEPTED = 1

# Error types
uint8 NO_ERROR = 0
uint8 DIFFERENT_ACTION_IS_RUNNING_ERROR = 1
uint8 INVALID_GOAL_ERROR = 2

string caller_id
uint8 action_type
string action_name
uint8 status_type
string status
uint8 error_type
string error_msg

