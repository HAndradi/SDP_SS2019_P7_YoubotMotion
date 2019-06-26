# Status types
uint8 STATUS_TYPE_FAILED = 0
uint8 STATUS_TYPE_SUCCEEDED = 1
uint8 STATUS_TYPE_PREEMPTED = 2

#Error types
uint8 ERROR_TYPE_NONE = 0
uint8 ERROR_TYPE_CONTROLLER_FAILURE = 1
uint8 ERROR_TYPE_INVALID_POSE_NAME = 2

string caller_id
youbot_motion_interface/Action_Info arm_action
youbot_motion_interface/Action_Info base_action
uint8 status_type
string status_msg
uint8 error_type
string error_msg

