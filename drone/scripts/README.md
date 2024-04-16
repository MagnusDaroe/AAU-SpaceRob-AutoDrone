# Scripts
The following scripts are used in the drone project: fc_cmd, image_listen, image_publish, manual_cmd and GCS

### fc_cmd
The fc_cmd script is used to send commands to the flight controller, by listening on the commands published from manual_cmd or a furture autonomous control script.

### manual_cmd
The manual_cd script is a scripts to read and send commands from a remote controller to the flightcontroller, by publishing its commands to the ros network.

### GCS
The GCS script provides a graphical user interface (GUI) for interacting with the system. It allows users to visually see the states of the drone, and where its heading.

