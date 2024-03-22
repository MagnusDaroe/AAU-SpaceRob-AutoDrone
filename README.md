# P4_DroneProject

<p align="right"> code inspired by [ArduSub](https://www.ardusub.com/developers/pymavlink.html).</p> <br>

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
Run **ReadAttitudeAndAccParam.py**<br>
Change port adress to your specified connection<br>
Prints **Roll**, **Pitch**, **Yaw**, **X-**, **Y-**, **Z-** **Acceleration** and **Gyroscope** data in realtime, example:<br>
*"Roll: 2.2466227475107945 deg, Pitch: 0.7973645012786337 deg, Yaw: -151.5053027351202 deg"* <br>
*"Acceleration: X: 0.18 m/s^2, Y: -0.32 m/s^2, Z: -9.92 m/s^2"* <br>
*"Gyroscope: X: 57.3 deg/s, Y: 0.0 deg/s, Z: -343.77 deg/s"* <br>
