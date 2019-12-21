---
layout: page
title: Results
nav_order: 4
description: >-
    Results of smoothiebot project
---

# Results
{:.no_toc}

## Table of contents
{: .no_toc .text-delta }

1. Performance
2. Demo
{:toc}

---

## Performance

As seen in the video, the SmoothieBot begins by prompting the user for a recipe which defines what fruit are desired. Once the transformations are captured between the RealSense camera, the table’s AR tags, and the Baxter robot, the fruit are placed on the table in front of the camera. The point cloud is captured and used to classify the fruit and the system picks a fruit that is specified in the recipe to target. From there, the robot reaches forward, grabs the fruit, and drops it in the blender. This process, starting at capturing the table’s pointcloud, is repeated for every fruit in the recipe.  Once the recipe is complete and all desired fruit have been added to the blender, the robot grabs the blender lid and drops it onto the blender. 

## Demo

[![demo](https://img.youtube.com/vi/gyzpliYOKKU/0.jpg)](https://www.youtube.com/watch?v=gyzpliYOKKU)

[![demo](https://img.youtube.com/vi/VEV4oVUFBEc/0.jpg)](https://www.youtube.com/watch?v=VEV4oVUFBEc)

