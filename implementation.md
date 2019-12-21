---
layout: page
title: Implementation
nav_order: 3
description: >-
    Implementation of smoothiebot project
---

# Implementation
{:.no_toc}

## Table of contents
{: .no_toc .text-delta }

1. TOC
{:toc}

---
## Code Structure
We separated our code into three main modules: `SmoothieMaster.py`, `FruitClassifier.py`, `FruitPicker.py`. The `SmoothieMaster` class was the main driver code for our project that controlled the high level flow of our algorithm. `FruitClassifier` handles the classification step of the algo. After this class is instantiated in `SmoothieMaster`, it classifies fruit on the table and stores relevant data for each fruit in a `Fruit.py` class. Each fruit object contained attributes such as fruit type, centroid coordinates, principal components, and point cloud data. These fruit objects then passed into the next module, `FruitPicker`, which handles the logic behind path planning and gripper actuation for picking up and moving fruit and the lid.

## Initialization

At the initialization of SmoothieBot, several packages are launched including the MoveIt package. In addition to this, the transformations between the RealSense camera, the AR tags placed on the table, and the Baxter robot are captured using the tf package. Because only one type of AR tracker can be run at once, the program must capture the transformations associated with the Baxter first, stop the generic AR Tracker, start the AR Tracker specific to the Realsense camera, and finally capture the transformation associated with the RealSense camera. These transformations are assumed to stay constant throughout the smoothie making process. Once all transformations are captured, the user is prompted for a recipe and can either select from premade recipes that are hard-coded in or create their own custom recipe. Opting for the customized options prompts the user to specify how many of each fruit they want. 

In order, the commands we run are:

1. In terminal (1): `rosrun baxter_interface joint_trajectory_action_server.py`

2. In terminal (2): `roslaunch baxter_moveit_config baxter_grippers.launch`

3. In terminal (3): `roslaunch realsense2_camera rs_camera.launch filters:=pointcloud`

4. In terminal (4): `roslaunch lab4_cam ar_track.launch`

5. In terminal (5): `rosrun rviz rviz`

6. In terminal (6): `rosrun tf static_transform_publisher 0 0 0 0 0 0 1 reference/left_hand_camera left_hand_camera 10` (Ayrton only)

7. In terminal (7): `rosrun SmoothieMaster.py`

8. In terminal (4): `roslaunch lab4_cam ar_track_realsense.launch` # Note only one type of AR tracker running at a time


## Capturing the Pointcloud 

After a desired recipe is specified, various fruits are placed on the table in front of the RealSense camera. The fruit can be placed on top of the AR tag on the table since the transformation has already been captured. The RealSense then takes a snapshot of the pointcloud of all of the fruit as well as the table. Encapsulated in that point cloud are the translational coordinates relative to the camera as well as RGB color data. A snapshot from the RealSense is taken between each attempt to pick up a fruit in case positions change in between grasps.

<img src="../pictures/pointcloud1.png" alt="Flow Diagram"/>
<img src="../pictures/pointcloud2.png" alt="Flow Diagram"/>

## Clustering Fruit 

Applying the transformation to the point cloud to the camera, calculated during initialization, allows for a leveled cloud in the frame of the camera. Following the transformation, a median of the z axis was taken, and then used as a mask for the point cloud - everything below the median (essentially all points representing the table) is then removed. The remaining points then shown ideally only included those pertaining to the fruit.
In order to identify each fruit as their own individual cluster, a density-based spatial clustering of applications with noise (DBSCAN) algorithm was implementation. Relying on two parameters - the maximum distance between two points pertaining to the same cluster, and minimum number of points required to be classified as an individual cluster - each point meeting these requirements was then labeled to a cluster. A cluster of points pertaining to each fruit was then passed out to our fruit classifier.


## Classifying Fruit 

Each cluster of fruit points is then passed through our fruit classifier, and has its principal components computed. If the explained variance of its first principal component is greater than our precomputed “banana threshold”, then the cluster will be classified as a banana, since it is the only non-spherical fruit. If otherwise, then the remaining clusters pass on to the RGB classifier.
Finally, an average of the RGB vector of the clusters is computed and compared to a preset list of vectors pertaining to each of the possible fruit in our recipe list. The Manhattan distance between the average RGB vector and predetermined fruit in the list is computed, and minimum distance classifies the cluster as one of our possible fruits. 

<img src="../pictures/bananapc.png" alt="Flow Diagram"/>
<img src="../pictures/bananapc2.png" alt="Flow Diagram"/>


## Path Planning

Once a fruit is deemed desirable, its centroid is calculated and the system uses the tf package to look up the transform from the base of Baxter to the centroid of the fruit. The translational vector of this transformation is entered into a pose message. The quaternion of the pose message is set such that the gripper is facing downward. The Baxter arm first moves to a “reset” position, close to the chest of the Baxter, then uses the pose message to move a little above the fruit, downwards to grab it, then back to the reset position. 

The location of the blender and the blender lid are saved from the transformations captured during initialization. The blender itself has an AR tag taped to its base such that the offset from the AR tag to the blender opening can be hardcoded in. Thus, using this transformation from the base to the blender’s tag, plus the offset from the tag to the blender opening, the program is able to construct a pose message to move the fruit to the blender opening.
Similarly, the blender lid is placed at the same location relative to an AR tag placed on the table. When the recipe is complete and all fruits have been added to the blender, the program knows to grab the blender lid and move it to the blender opening. The same pose is used to move the fruit and the blender lid to the blender opening. 

