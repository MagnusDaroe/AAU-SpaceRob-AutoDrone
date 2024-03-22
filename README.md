# P4_DroneProject

code inspired by https://www.ardusub.com/developers/pymavlink.html

## Test of motors with manual control
Stabilize-arm-motor_test.py
Change port adress to your specified connection
The code prints "COMMAND_ACK {command : "sent command", result : 0, progress : 0, result_param2 : 0, target_system : 255, target_component : 0}"
If "result" is 0, then the command is succesfull, where 4 is declined. Else look in the documentation: https://mavlink.io/en/services/command.html

## Test of arming the drone
Change port adress to your specified connection
Run ModeAndArm-confirmation.py
The code prints the states of the drone.

## Read attitude parameters
Run ReadAttitudeParam.py
Change port adress to your specified connection
Prints Roll, Pitch and Yaw in realtime, example:
Roll: 2.1942973086095616, Pitch: 1.3729335502999127, Yaw: -155.02535000691302
