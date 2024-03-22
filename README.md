# P4_DroneProject

code inspired by [ArduSub](https://www.ardusub.com/developers/pymavlink.html).<br>

### Test of motors with manual control
Run **Stabilize-arm-motor_test.py**<br>
Change port adress to your specified connection<br>
The code prints *"COMMAND_ACK {command : "sent command", result : 0, progress : 0, result_param2 : 0, target_system : 255, target_component : 0}"*<br>
If "result" is 0, then the command is succesfull, where 4 is declined. Else look in the documentation: [Mavlink.io](https://mavlink.io/en/services/command.html)<br>

### Test of arming the drone
Run **ModeAndArm-confirmation.py**<br>
Change port adress to your specified connection<br>
The code prints the states of the drone.<br>

### Read attitude parameters
Run **ReadAttitudeParam.py**<br>
Change port adress to your specified connection<br>
Prints Roll, Pitch and Yaw in realtime, example:<br>
*"Roll: 2.1942973086095616, Pitch: 1.3729335502999127, Yaw: -155.02535000691302"*<br>
