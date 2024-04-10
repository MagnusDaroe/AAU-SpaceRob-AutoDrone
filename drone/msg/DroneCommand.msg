# commands which both modes can use
float64 timestamp
uint8 cmd_estop

# manual commands
uint8 cmd_arm
int32 cmd_mode 
uint8 cmd_reboot

float32 cmd_roll
float32 cmd_pitch
float32 cmd_yaw
float32 cmd_thrust

# autonomous commands
float32 cmd_auto_roll
float32 cmd_auto_pitch
float32 cmd_auto_yaw
float32 cmd_auto_thrust

