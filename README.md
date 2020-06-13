# Project obj-lst-vis user manual
Developement of an autonomous driving environment model visualization based on object list level

-- Student project at Technische Hochschule Ingolstadt (03/2020 - 07/2020) -- 

This repository can be cloned and used as catkin workspace in a ROS environment 

## CARLA

## YOLO and ROS

## Post-processing

### GUI
0. Start GUI:
	* Start roscore in a new terminal (if it is not already running):
	``` bash
	roscore
	```
	* Setup your environment (in a new terminal, in workspace *obj-lst-vis*):
	``` bash
	./devel/setup.bash
 	 ```
	* Start rqt (in the same terminal as step b.):
	``` bash
	rqt
    ```
	* Start the post-processing Plugin: Select "ObjectList Postprocessing Plugin" under "Plugins"
  
1. Import the Rosbag files to investigate
	* Import a Ground Truth Rosbag file by pressing the button "ground truth bag file" and selecting one in the folder structure
	* Import a Camera Data Rosbag file by pressing the button "camera data bag file" and selecting one in the folder structure
  
2. Add graphs to the plot area by pressing the button "Add new Graph":
	* Tab "Raw Data Graphs":
   		* Select the desired Rosbag file (only one selectable)
     	* Select the desired value which should be investigated
    	* Select a ObjectID by clicking one in the list or by typing a number in the edit field
	* Tab "Evaluation Graphs":
    	 * Select the desired evaluation value
    	 * optional: adjust the threshold value for the IoU evaluation
	* Tab "Difference Graphs":
    	* optional: adjust the threshold value for the IoU evaluation
     	* Select the desired value which should be compared
    	* Select a ObjectID (here: always GT-IDs) by clicking one in the list or by typing a number in the edit field
		
  After selecting the desired graph press "Start" and the graph is shown in the plot area of the main window

3. Delete a graph by pressing the button "Delete Graph", selecting the graph in the list and pressing the button "Delete"

4. Get the quality parameters FPPI, MOTP and MOTA of the imported camera data Rosbag file in comparion to the imported Ground Truth Rosbag file by pressing the button "Compute Data Quality". The IoU threshold can be adjusted and the values recalculated by pressing the button "Recalculate Quality"
  
