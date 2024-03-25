# Graspnet Skill
Prompt guided object picking as a skiros2 skill.

## About
This repository contains an implementation of Contact Graspnet and Grounded SAM in ROS1. This skill uses the Grounded SAM model to detect the object and executes the picking task using the Contact_Graspnet model and Moveit package.

## Compatibility
- Tested on ROS Noetic, might work with other ROS distributions.

## Requirements
- ROS1
- skiros2 ([skiros2](https://github.com/RVMI/skiros2))
- contact_graspnet ([contact_graspnet](https://github.com/HashimHS/contact_graspnet))
- grounding_sam_ros ([grounding_sam_ros](https://github.com/HashimHS/grounding_sam_ros))
- moveit ([moveit](https://moveit.ros.org/install/))

## Installation
After installing the required packages above, follow these steps:

1. Clone this repository:
    ```bash
    cd ~/catkin_ws/src
    git clone
    ```

2. Add the skills to skiros2 by modifying the skill_mgr.launch file:
```
    <arg name="libraries_list" value="[graspnet_skill]"/>
    <arg name="skill_list" value="[detect_graspnet, move_graspnet]" />
```
You can also refer to this ([guide](https://github.com/RVMI/skiros2/wiki/Tutorial-2:-Launch-system#skill-manager)) on how to add your skills to skiros2.

3. Change the frame names in manipulation.py and graspnet.py according to your robot configuration.
graspnet.py:
```
CAMERA_FRAME = 'realsense_rgb_optical_frame'
BASE_LINK = 'ur5e_base_link'
END_EFFECTOR_LINK = 'ur5e_ee_graspnet'
```
manipulation.py:
```
BASE_LINK = 'ur5e_base_link'
END_EFFECTOR_LINK = 'ur5e_ee_graspnet'
MOVE_GROUP = 'manipulator'
```

4. Build the ROS workspace:
    ```bash
    # Navigate to the root of your ROS workspace
    cd ~/catkin_ws
    # Build the workspace
    catkin build
    ```

## Usage
Launch skiros2 gui using the command:
```
roslaunch skiros2 skiros.launch 
```

## Contributors
This ROS package is made possible by:
- Hashim Ismail ([HashimHS](https://github.com/HashimHS)).
- JLL ([Taokt](https://github.com/Taokt)).