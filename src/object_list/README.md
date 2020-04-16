# Object_List Package

This package include the proposed Object List [ROS messages](https://github.com/MaikolDrechsler/Object_List/tree/master/src/object_list/msg), and a [Dummy node](https://github.com/MaikolDrechsler/Object_List/tree/master/src/object_list/scripts) to generate this message

# Installation

- Install [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Create your workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
- Download the repository in your workspace
- Open your workspace directory in the terminal and build the packages 

```bash
catkin_make
```

# Running node

Once built the packages you can run the node: 

## Object List Dummy

```bash
rosrun object_list sensor_model_dummy.py
```
