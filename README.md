

# Building the catkin workspace
It is recommended to configure the following after running `catkin init`
```
catkin config --skiplist dvs_ros_driver dvxplorer_ros_driver dvs_file_writer --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## NOTE: 
maybe also `--merge-devel`