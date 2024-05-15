# commands which both modes can use
float64 timestamp
uint8 cmd_estop
uint8 cmd_eland
uint8 identifier

# manual commands
uint8 cmd_arm
int32 cmd_mode 

int32 cmd_roll
int32 cmd_pitch
int32 cmd_yaw
int32 cmd_thrust

# autonomous commands
int32 cmd_auto_roll
int32 cmd_auto_pitch
int32 cmd_auto_yaw
int32 cmd_auto_thrust
uint8 cmd_auto_disarm

