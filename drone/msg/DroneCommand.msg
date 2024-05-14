# commands which both modes can use
float64 timestamp
uint8 cmd_estop
uint8 cmd_eland
uint8 identifier

# manual commands
uint8 cmd_arm
int32 cmd_mode 

float32 cmd_roll
float32 cmd_pitch
float32 cmd_yaw
float32 cmd_thrust

# autonomous commands
float32 cmd_auto_roll
float32 cmd_auto_pitch
float32 cmd_auto_yaw
float32 cmd_auto_thrust
uint8 cmd_auto_disarm

