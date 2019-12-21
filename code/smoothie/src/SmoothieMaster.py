#!/usr/bin/env python
import rospy
import tf2_ros
import sys
from geometry_msgs.msg import Twist
import tf2_geometry_msgs

from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs.msg import PointCloud2
import ros_numpy
import numpy as np

from scipy.spatial.transform import Rotation as R
import tf.transformations as tr
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy import signal
import sys

from FruitClassifier import Fruit, FruitClassifier
from FruitPicker import FruitPicker
from Recipe import Recipe

# import grasp_planner
# import Recipe

# Setup instructions
# 1. Navigate to smoothieBot directory
# 2. ssh into robot using: "./baxter.sh [name-of-robot].local"
# 3. Reset arms of robot using: "rosrun baxter_tools tuck_arms.py -u"
# 4. From ssh terminal run: rviz rviz
# 5. open new terminal and run catkin_make
# 6. (this doesn't look like it works or does anything) in that terminal run roslaunch lab4_cam run_cam.launch
# 7. in that terminal run roslaunch lab4_cam ar_track.launch (for launching ar tracking with the baxter)
# 8. open a new terminal and run "catkin_make" and "source ~/ros_workspaces/smoothieBot/devel/setup.bash"
# 9. in that new termainl run "roslaunch realsense2_camera rs_camera.launch filters:=pointcloud"
# 9.5 also run roslaunch lab4_cam ar_track_realsense.launch (for launching ar tracking with the realsense)
# 10. On rviz: change fixed frame to be "camera_depth_optical_frame"
# 11. add a new display of type pointcloud2
# 12. for ar tracking with realsense: fixed frame is camera_color_optical_frame

## INSTEAD OF 7 & 9.5 : run roslaunch lab4_cam ar_track_both.launch

# 4. Launch realsense2_camera: "roslaunch realsense2_camera rs_camera.launch filters:=pointcloud"
# 5. Open new terminal, launch ar_trackers: "roslaunch lab4_cam ar_track_both.launch"
# 6. run python SmoothieMaster.py

# EXTRA COMMANDS:
# Enable joints by running rosrun baxter_tools enable_robot.py -e (Sawyer: rosrun intera_interface enable_robot.py) (not sure if we actually need this, need to double check)
# Open right hand camera at maximum frame height and width: rosrun baxter_tools camera_control.py -o right_hand_camera -r 1280x800
# View camera feed: rosrun image_view image_view image:=/cameras/right_hand_camera/image
# 8. rosrun baxter_interface joint_trajectory_action_server.py
# 9. roslaunch baxter_moveit_config baxter_grippers.launch
# 10. rosrun baxter_tools camera_control.py -o left_hand_camera -r 1280x800
# 11. rosrun tf static_transform_publisher 0 0 0 0 0 0 1 reference/right_hand_camera left_hand_camera
# How to Use:
# 1. python moothieMaster.py
# 2. Prompt user to enter a recipe
# 3. 

class SmoothieMaster(object):

	def __init__(self, gripper_arm="right"):
		print("Initializing SmoothieMaster...")
		rospy.init_node('SmoothieMaster', anonymous=True)

		self.gripper_arm = gripper_arm

		# All AR tags we will need
		self.ar_marker_tag = "ar_marker_6"
		self.blender_ar = "ar_marker_1"
		self.lid_ar = "this is not ready yet"
		#finding the transformation from the ar_tag on the able to the base of the robot
		self.ar_to_base = self.find_transform("base",self.ar_marker_tag)
		self.g_ar_to_base = self.msg_to_se3(self.ar_to_base)
		print("hi")
		#finding the transformation from the real sense camera to the ar_tag on the table
		self.realsensecamera_to_ar = self.find_transform(self.ar_marker_tag,"camera_color_optical_frame")
		print("found transform")
		self.g_realsense_to_ar = self.msg_to_se3(self.realsensecamera_to_ar)
		print("2")
		print("SmoothieMaster Initialization Complete!!!\n")

	def main(self):
		"""
		1. Calibration (done in initialization of smoothieMaster)
			- get transform from robot base link to table
			- get transform from realsense camera to table
		2. Instantiate Recipe.py and prompt user for order. Access ingredients using recipe.getorder()

		for fruit_desired in ingredient list
			3. Get pointcloud, transform it to table's ar-tag frame
			4. give transformed pointcloud to a fruitClassifier object
			5. fruitClassifier has a list of fruit (fruit_list) in the frame of the table's ar-tag
			6. find fruit_desired in fruit_list
			7. in smoothieMaster, transform fruit_desired's centroid and pc to the base frame of the robot
			8. instantiate fruitPicker with fruit_desired object and AR-tag string of blender
			8.5 run fruitpicker.movetofruit, fruitpicker.grabfruit, fruitpicker.movetoblender

		9. close the gd lid
		"""
		recipe = Recipe()
		order = recipe.get_order() # order is a dictionary of fruits and the quantity of each
		for fruit_desired in order:
			#Gets snapshot of PointCloud2 data from '/camera/depth/color/points' topic
			print("order: ", order)
			print("fruit desired:", fruit_desired)
			self.get_pointcloud()
					
			# converts pointcloud2 message to nparray, split xyz and rgb data
			pc_xyz, pc_rgb = self.split_xyz_rgb()

			# homogonize xyz array
			ones = np.ones(pc_xyz.shape[1])
			ones = np.reshape(ones, (1,pc_xyz.shape[1]))

			# apply transformation matrix to the xyz values of the full table pointcloud
			xyz_homog = np.dot(self.g_realsense_to_ar,np.vstack((pc_xyz,ones)))

			# re-append rgb values to the transformed xyz values, creating 
			table_pointcloud = np.vstack((xyz_homog[:3,:], pc_rgb)).T

			# input point cloud into fruit classifier object, finds list of available fruit 
			fruit_classifier = FruitClassifier(table_pointcloud)
			available_fruit = fruit_classifier.get_fruits()

			for fruit in available_fruit:
				if fruit.get_fruit_type() == fruit_desired:
					target_fruit = fruit
					break

			self.add_transform_attributes(target_fruit)

			fruit_picker = FruitPicker(target_fruit, self.blender_ar, gripper_arm)
			fruit_picker.move_to_fruit()
			fruit_picker.grab_fruit()
			fruit_picker.move_to_blender()

		# close lid

			# # For debugging-- plots the point cloud
			# thresh_cloud = self.filter_table(cloud_test)

			# x = thresh_cloud[:,0]
			# y = thresh_cloud[:,1]
			# z = thresh_cloud[:,2]

			# fig = plt.figure()
			# ax = fig.add_subplot(111, projection='3d')
			# ax.scatter(x,y,z, marker='.')
			# plt.show()


		# 	#TODO: get pointcloud of fruit and table lab6
		# 	pc = ...

		# 	#TODO: transform points to table AR tag frame of reference, filter out table pointcloud points
		# 	thresh = 0
		# 	trans_points = trans.apply(pc)
		# 	filtered_points = [pt for pt in trans_points if pt[2] > thresh] #takes out the table

		# 	#put in fruits pointcloud points into FruitClassifier and get out list of Fruit objects	

		# 	fruit_classifier = FruitClassifier(filtered_points)
		# 	allFruit = fruit_classifier.get_fruits() #list of Fruit objects -- contains a string of what fruit and the pointcloud attributed to that fruit
			
		# 	#loc = allFruit[fruit]
		# 	picker.grab(loc)
		return

	def find_transform(self, start_frame, goal_frame):
		#find_transform returns TransformStamped, transform is contained in a Transform type
		#Transform contains Vector3 translation and Quaternion rotation

		#To access translation: do realsensecamera_to_ar.transform.translation.{x,y,z}
		#To access quaternion: do realsensecamera_to_ar.transform.rotation.{x,y,z,w}

		tfBuffer = tf2_ros.Buffer()
		tfListener = tf2_ros.TransformListener(tfBuffer)
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			try:
				trans = tfBuffer.lookup_transform(start_frame, goal_frame, rospy.Time())
				return trans
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				print(e)
				pass
			r.sleep()

	# get snapshot of pointcloud2 message from '/camera/depth/color/points' topic
	def get_pointcloud(self):
		self.foundpc = False

		self.sub = rospy.Subscriber('/camera/depth/color/points',PointCloud2,self.callback)
		while self.foundpc == False: 
			rospy.rostime.wallsleep(0.5)

	def callback(self,msg):
		print('callback')
		self.pointcloud = msg
		self.foundpc = True
		self.sub.unregister()
		print('done')
		return

# OBSOLETE -- METHOD MOVED TO FruitClassifier.py
	# def filter_table(self, cloud):
	# 	x = cloud[:,0]
	# 	y = cloud[:,1]
	# 	z = cloud[:,2]
	# 	r = cloud[:,3]
	# 	g = cloud[:,4]
	# 	b = cloud[:,5]

	# 	z_med = np.median(z)
	# 	z_mask = z > z_med + .005

	# 	mx = np.asarray(x[np.nonzero(z_mask)])
	# 	my = np.asarray(y[np.nonzero(z_mask)])
	# 	mz = np.asarray(z[np.nonzero(z_mask)])
	# 	mr = np.asarray(r[np.nonzero(z_mask)])
	# 	mg = np.asarray(g[np.nonzero(z_mask)])
	# 	mb = np.asarray(b[np.nonzero(z_mask)])

	# 	thresh_xyz = np.asarray(list(zip(mx,my,mz)))
	# 	thresh_rgb = np.asarray(list(zip(mr,mg,mb)))
	# 	thresh_pointcloud = np.hstack((thresh_xyz,thresh_rgb))
	# 	return thresh_pointcloud


	def split_xyz_rgb(self):
		#input: an pointcloud2 message type
		#output: xyz (nparray of the xyz coordinates)
		#		 rgb (nparrray of the rgb values)-
		points = ros_numpy.point_cloud2.pointcloud2_to_array(self.pointcloud)
		pc_tuple = ros_numpy.point_cloud2.split_rgb_field(points)
		pc = np.asarray([list(p) for p in pc_tuple])
		rgb = pc[:,3:].T
		xyz = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(self.pointcloud).T
		return xyz, rgb
	
	def msg_to_se3(self, msg_stamped):
		"""Conversion from geometric ROS messages into SE(3)
		@param msg: {geometry_msgs/TransformStamped}
		@return: a 4x4 SE(3) matrix as a numpy array
		@note: Throws TypeError if we receive an incorrect type.
		"""
		try:
			# p, q = transform_stamped_to_pq(msg)
			msg = msg_stamped.transform
			p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
			q = np.array([msg.rotation.x, msg.rotation.y,
						  msg.rotation.z, msg.rotation.w])
		except Exception as e:
			pass
			# raise TypeError("please only input a geometry_msgs/TransformStamped")

		norm = np.linalg.norm(q)
		if np.abs(norm - 1.0) > 1e-3:
			raise ValueError(
				"Received un-normalized quaternion (q = {0:s} ||q|| = {1:3.6f})".format(
					 str(q), np.linalg.norm(q)))
		elif np.abs(norm - 1.0) > 1e-6:
			q = q / norm
		g = tr.quaternion_matrix(q)
		g[0:3, -1] = p
		return g


	def add_transform_attributes(self,fruit):
		#TODO: takes in a fruit in the table's ar-tag frame, transforms it's centroid and 1st principal component into the robot's base frame
		cent = fruit.get_centroid()
		cent_homo = np.array([cent[0],cent[1],cent[2],1])
		cent_trans = np.dot(self.g_ar_to_base,cent_homo)

		pc1 = fruit.get_pc1()
		pc1_homo = np.array([pc1[0],pc1[1],pc1[2],1])
		pc1_trans = np.dot(self.g_ar_to_base,pc1_homo)

		pc2 = fruit.get_pc2()
		pc2_homo = np.array([pc2[0],pc2[1],pc2[2],1])
		pc2_trans = np.dot(self.g_ar_to_base,pc2_homo)

		pc3 = fruit.get_pc3()
		pc3_homo = np.array([pc3[0],pc3[1],pc3[2],1])
		pc3_trans = np.dot(self.g_ar_to_base,pc3_homo)

		fruit.transformAttributes(cent_trans, pc1_trans, pc2_trans, pc3_trans)

		return


if __name__ == '__main__':
	master = SmoothieMaster()
	master.main()




