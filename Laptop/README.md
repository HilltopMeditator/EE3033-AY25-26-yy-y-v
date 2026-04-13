# About this folder
Scripts and catkin workshops to be built and run on the host computer

## Laptop_commander.py

Top level Manager for Mapping phase. Utilizes a state machine to coordinate node launching.

## finalproject_ws (Image Detection & Maze Exploration)

This workspace contains `darknet_ros` for YOLO-based object detection and `maze_explore` for milestone execution. 

Before building, ensure the Python scripts are executable:
```bash
chmod +x ~/ee3033-ay25-26-yy-y-v/Laptop/finalproject_ws/src/maze_explore/scripts/milestone_one.py
```

Build/Source command:

```
cd Laptop/finalproject_ws
rm -rf build/ devel/
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash ## change .sh to .zsh or .fish if required
```