

uint8 DEFAULT = 0 
uint8 ARM_ACTION_TYPE_LOC_NAME = 1 
uint8 ARM_ACTION_TYPE_POSE = 2
uint8 ARM_ACTION_TYPE_JOINTS = 3
uint8 ARM_ACTION_TYPE_VELOCITY = 4 

bool stop_current
geometry_msgs/PoseStamped goal_pose 
brics_actuator/JointPositions joint_angles 
std_msgs/String pose_name

uint8 arm_action_type 

