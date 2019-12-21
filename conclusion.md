---
layout: page
title: Conclusion
nav_order: 5
description: >-
    Conclusion of smoothiebot project
---

# Conclusion
{:.no_toc}

## Table of contents
{: .no_toc .text-delta }

1. Summary
2. Difficulties
3. Future Improvements

{:toc}

---

## Summary

Overall, the project was successful in that, after a recipe was input by the user, Baxter was able to independently recognize the fruits on the table, pick them up, move them to the blender, and close the blender lid. Looking at the captured point clouds, it was clear that the project was accurate in transforming the points and filtering out the table. The classification system was also quite effective, able to accurately recognize the fruit placed in front of the camera, given that the RealSense was appropriately positioned not too far away.

## Difficulties

We noticed that despite calibrating the robot with offset values to pick up the fruit at the centroid, the robot would seem to be offset by different values each time we restarted our work (ie coming back to work on another day). We realized that we needed to standardize the position of the Baxter’s non gripper hand (in our case, left hand camera) as well as the position of the RealSense relative to the table AR tag in order to achieve replicable results. We achieved this by designating an initialization position and calculating the joint angles for us to use during the initialization sequence. We also standardized our placement of the RealSense relative to the AR tag.

In addition, we noticed that between different robots: Ayrton and Archytas for example, that the tf_frame convention was different. On ayrton, frames were named as “reference/left_hand camera” as opposed to just “left_hand_camera.” By analysis of the tf_frame diagram and executing a static transform publisher, we were able to resolve this.

Finding the centroid for fruits with complex shapes was also more complicated than expected. In reality, we didn’t really want the exact centroid, we wanted the position above the table that would be the ideal spot to grab the fruit. For spherical fruits this was the same as the centroid. For a curved banana, however, the centroid would have actually been offset from the ideal gripping point of the banana (and possibly even outside of the banana if it was curved enough). To account for this, from the pointcloud, we first found each point’s average value along the first principal component and took points that were within a range of values close to the average. From those points, we found their value when projected onto the second principal component, took the average, and ultimately converted this to an x, y position which we used as the banana’s centroid.


## Future Improvements

The grabbing motion was not as versatile as we had initially hoped for. The robot is able to successfully move its gripper above the desired fruit and reach down to grab it. However, some fruits must be grabbed at a certain orientation in order to fit in the gripper. For example, a banana cannot be grasped length-wise. To fix this, we made sure to orient the fruit such that the gripper could more easily grab them. We did preliminary work using PCA to find the angle of the fruit’s first principal component relative to the table AR tag (which could then be used to set the joint angle of Baxter’s gripper) but unfortunately did not get around to testing this before the demo. Further improvements can be made to enable Baxter to pick up fruit not only from a top down position, but also to approach fruits from the side. This however, would also require work on collision detection, such that Baxter’s planned path doesn’t collide with other fruits or the blender.

Another idea is to design and 3D print hardware that would enable baxter to consistently grip the lid and seal the blender, as opposed to using parallel grippers which causes the lid to swing during movement. Additional work on the lid closing program can also be done to ensure a sufficiently tight seal.

