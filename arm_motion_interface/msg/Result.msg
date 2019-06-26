# Arm motion action types
uint8 DEFAULT = 0
uint8 ACTION_TYPE_MOVEIT_NAME = 1
uint8 ACTION_TYPE_MOVEIT_POSE = 2
uint8 ACTION_TYPE_MOVEIT_JOINTS = 3
uint8 ACTION_TYPE_CVC_POSE = 4

# Status types
uint8 ACTION_FAILED = 0
uint8 ACTION_SUCCEEDED = 1
uint8 ACTION_PREEMPTED = 2

string caller_id
uint8 action_type
string action_name
uint8 status_type
string status

