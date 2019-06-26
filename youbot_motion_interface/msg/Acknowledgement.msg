#Status types
uint8 STATUS_TYPE_REJECTED = 0
uint8 STATUS_TYPE_ACCEPTED = 1

#Error types
uint8 ERROR_TYPE_NONE = 0
uint8 ERROR_TYPE_INVALID_ACTION_TYPE = 1
uint8 ERROR_TYPE_CONFLICTING_ACTION_RUNNING = 2
uint8 ERROR_TYPE_ARM_CANNOT_BE_PREEMPTED = 3 

string caller_id
youbot_motion_interface/Action_Info arm_action
youbot_motion_interface/Action_Info base_action
uint8 status_type
string status_msg
uint8 error_type
string error_msg

