#Base motion action types
uint8 DEFAULT = 0
uint8 ACTION_TYPE_MOVE_BASE_POSE = 1
uint8 ACTION_TYPE_MOVE_BASE_NAME = 2
uint8 ACTION_TYPE_DBC_POSE = 3

#Status types
uint8 ACTION_FAILED = 0
uint8 ACTION_SUCCEEDED = 1
uint8 ACTION_PREEMPTED = 2

string caller_id
uint8 action_type
string action_name
uint8 status_type
string status

