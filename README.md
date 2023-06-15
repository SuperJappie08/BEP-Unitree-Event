
# Data set
The datasets can be found [here](https://drive.google.com/drive/folders/11Oc9271Foi5BNg3_TgCQCrz3SPWoiMqq?usp=sharing).

# Building the catkin workspace
It is recommended to configure the following after running `catkin init`
```
catkin config --skiplist dvs_ros_driver dvxplorer_ros_driver dvs_file_writer --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## NOTE: 
maybe also `--merge-devel`

Copy the camera calibration to `/root/.ros/camera_info/{file}` for WSL (root user) and to `/home/$USER/.ros/camera_info/{file}`

# RUN RECORD SCRIPT
```
. ./record.sh
```
or
```
source ./record.sh
```

# Requirements
- sshpass

```
sudo apt-get install sshpass
```
