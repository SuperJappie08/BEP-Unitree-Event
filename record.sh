#!/usr/bin/bash
RECORD_SCRIPT_ROS_CATKIN_WS="$(dirname -- "${BASH_SOURCE[0]}" )/catkin_ws/devel/setup.bash"


if [ -z "$ROS_DISTRO" ]
then
    echo -e "\033[0;33m[WARNING]\033[0m ROS workspace was not loaded."

    echo -n -e "\a > "
    read -p "Want to fix load environment? (source \"${RECORD_SCRIPT_ROS_CATKIN_WS}\") [Ny]" -n 1 -r REPLY
    echo    # (optional) move to a new line
    if [[ ! $REPLY =~ ^[Yy]$ ]]
    then
        echo -e "\033[0;31m[FAILED]\033[0m\a ROS is not ok, manual intervention chosen."
        [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
    else
        source $RECORD_SCRIPT_ROS_CATKIN_WS
    fi
else
    echo -e "\033[1;32m[OK]\033[0m ROS Loaded"
fi

if [ $ROS_MASTER_URI != "http://192.168.12.1:11311/" ] && [ $HOSTNAME = "raspberrypi" ] && [ $USER = "pi" ] && [ -e "$HOME/network_bridge.sh" ]
then
    echo "This is running on the robot"
else
    if [ "$ROS_MASTER_URI" != "http://192.168.12.1:11311/" ]
    then
        echo -e "\033[0;33m[WARNING]\033[0m The ROS_MASTER_URI was set incorrectly! (Now ${ROS_MASTER_URI})"
        if [ -z "$REPLY" ]
        then
            echo -n -e "\a > "
            read -p "Want to fix ROS_MASTER_URI? [Ny]" -n 1 -r REPLY
            echo    # (optional) move to a new line
        fi

        if [[ ! $REPLY =~ ^[Yy]$ ]]
        then
            echo -e "\033[0;31mFAILED\033[0m\a ROS_MASTER_URI requested to be unchanged."
            [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
        else
            export ROS_MASTER_URI="http://192.168.12.1:11311/"
            echo "\033[1;32m[OK]\033[0m Changed ROS_MASTER_URI to ${ROS_MASTER_URI}"
        fi
    fi
fi

# RECORD_SCRIPT_DOG_TEST=$(timeout -s SIGTERM 1 rostopic echo /camera1/range_visual_front)
# if [ ! $( echo $RECORD_SCRIPT_DOG_TEST | wc -l ) -gt 1 ]
# then
#     echo -e "\033[0;33m[WARNING]\033[0m The network was not bridging has not been configured"
#     if [ -z "$REPLY" ]
#     then
#         echo -n -e "\a > "
#         read -p "Want to fix network? [Ny]" -n 1 -r REPLY
#         echo    # (optional) move to a new line
#     fi

#     if [[ ! $REPLY =~ ^[Yy]$ ]]
#     then
#         echo -e "\033[0;31m[FAILED]\033[0m\a Network of Robot not bridged."
#         [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
#     else
#         if ping -c 1 -W 1 -q 192.168.12.1 > /dev/null;
#         then
#             echo "\033[1;33m[Ok]\033[0m Found Robot, bridging network."
#             sshpass -p "123" ssh -o StrictHostKeyChecking=no pi@192.168.12.1 "./network_bridge.sh"
#         else
#             echo -e "\033[1;33m[TESTING]\033[0m Robot was not found on the network. Continuing for testing"
#         fi
#     fi
# else
#     echo -e "\033[1;32m[Ok]\033[0m Network is correct."
# fi

# RECORD_SCRIPT_DOG_TEST=$(timeout -s SIGTERM 1 rostopic echo /dvs/imu)
# if [ ! "$( echo $RECORD_SCRIPT_DOG_TEST | wc -l )" -gt 1 ]
# then
#     echo -e "\033[0;31m[FAILED]\033[0m The Event-Camera is not running"
#     echo -e "\033[0;31mMANUAL FIX REQUIRED:\033[0m\n\a > In a new terminal navigate to the catkin_ws folder and run the following commands\n\t1. sudo su root\n\t2. source ./devel/setup.bash\n\t3. roslaunch bep record.launch"
#     [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
# else
#     echo -e "\033[1;32m[Ok]\033[0m Camera Node is running."
# fi

# RECORD_SCRIPT_DOG_TEST=$(timeout -s SIGTERM 1 rostopic echo /bep_onboard/status)
# if [ ! "$( echo $RECORD_SCRIPT_DOG_TEST | wc -l )" -gt 1 ]
# then
#     echo -e "\033[0;33m[WARNING]\033[0m The control node is not running on the robot."
#     # if [ -z "$REPLY" ]
#     # then
#     #     read -p "Want to start it? [Ny]" -n 1 -r REPLY
#     #     echo    # (optional) move to a new line
#     # fi

#     # if [[ ! $REPLY =~ ^[Yy]$ ]]
#     # then
#     #     echo "Failed on control node"
#     #     [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
#     # else
#     #     # USE A REMOTE LAUNCHFILE: http://wiki.ros.org/roslaunch/XML/machine
#     #     # roslaunch REMOTE
#     # fi
#     echo -e "\033[0;31m[FAILED]\033[0m Contol node must be running on the robot."
#     echo -e "\033[1;31mMANUAL FIX REQUIRED\033[0m\a"
#     [[ "$0" = "$BASH_SOURCE" ]] && exit 1 || return 1 # handle exits from shell or function but don't exit interactive shell
# else
#     echo -e "\033[1;32m[Ok]\033[0m Control Node is running."
# fi

echo -n -e "\033[1;32m[Ok]\033[0m\a"
read -p " Press Enter to Start? [Duration (DEFAULT 60)] " -r DURATION

if [ $((DURATION + 0)) -eq 0 ]
then
    DURATION=60
else
    DURATION=$((DURATION + 0))
fi

rosbag record --duration=$DURATION -e "(/bep_controller/robot_state|/dvs/(.*)|/camera(.)/(.*))" -o bags/BEP-TEST -b 0
