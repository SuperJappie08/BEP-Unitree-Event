# bep_onboard_controller
This package contains 2 nodes:
 - (❌) `bep_onboard_controller_node`
    - Should subscribe to `/cmd_bep` and be controllable via teleop
 - (✔️) `bep_capture_node`
    - Publishes zero velocity messages to allow the running capturing of the internal data

# Starting caputure
Connect to Unitree Go1
```bash
your@machine:....$ ssh pi@192.168.12.1
```
Then on the robot
```bash
$ ./network_bridge.sh
$ cd go1_catkin
$ source devel/setup.bash
$ roslaunch bep_onboard_controller capture.launch
```
Press enter to start 