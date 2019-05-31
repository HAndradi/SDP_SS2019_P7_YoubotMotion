uint8 DEFAULT = 0
uint8 ARM_ACTION_TYPE_NAME = 1
uint8 ARM_ACTION_TYPE_POSE = 2
uint8 ARM_ACTION_TYPE_JOINTS = 3
uint8 ARM_ACTION_TYPE_VELOCITY = 4

uint8 arm_action_type
bool stop_current_action

std_msgs/String name
geometry_msgs/PoseStamped pose
brics_actuator/JointPositions joints
