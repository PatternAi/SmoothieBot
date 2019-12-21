#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
import sys
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
import traceback

import sys
from baxter_interface import Limb
from path_planner import PathPlanner
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from baxter_interface import gripper as robot_gripper
from FruitClassifier import Fruit

class FruitPicker:
	"""
	Grasp Planner for smoothieBot
	From smoothieMaster:
		8. instantiate fruitPicker with fruit_desired object and AR-tag string of blender
		8.5 run fruitpicker.move_to_fruit, fruitpicker.grab_fruit, fruitpicker.move_to_blender
	"""
	def __init__(self, fruit_desired, blender_ar, gripper_arm):
		"""
		fruit_desired: fruit object with points in reference frame of table_ar_tag, fruit_type, pc_all, and centroid
		blender_ar: AR-tag string of blender
		gripper_arm: "left" or "right" depending on the side that the gripper arm is on 
		"""
		self.fruit_desired = fruit_desired
		self.blender_ar =  blender_ar
		self.gripper_arm = gripper_arm
		self.planner = PathPlanner(gripper_arm)
		self.gripper = robot_gripper.Gripper(self.gripper_arm)
		print('Calibrating...')
		right_gripper.calibrate()
		rospy.sleep(2.0)
		rospy.init_node('FruitPicker')
		print("FruitPicker initialized")

	def move_to_fruit(self, offset=0.5):
		#moves the robot arm to directly above the fruit
		centroid = fruit.get_centroid()
		fruit_pose = self.get_fruit_pose(centroid, offset)

		while not rospy.is_shutdown():
			try:
				plan = self.planner.plan_to_pose(fruit_pose, [])
				raw_input("Press <Enter> to move the right arm to fruit pose: ")
				if not self.planner.execute_plan(plan):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break
			print("Moved to Fruit: ", fruit.get_fruit_type())

	def grab_fruit(self, raise_offset=0.05):
		#Assumes that gripper is directly above fruit, moves the gripper down to fruit centroid, grabs the fruit, moves back up a short distance
		centroid = fruit.getCentroid()
		fruit_pose = self.get_fruit_pose(centroid) # Get fruit pose with no offset
		# TODO: improve fruit_pose by rotating gripper along PC2
		# Moves gripper down to the exact location of the fruit's centroid
		while not rospy.is_shutdown():
			try:
				plan = self.planner.plan_to_pose(fruit_pose, [])
				raw_input("Press <Enter> to grab the fruit: ")
				if not self.planner.execute_plan(plan):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break

		#Close the right gripper
		rospy.sleep(1.0)
		print('Closing...')
		right_gripper.close()
		rospy.sleep(1.0)

		# Raise the fruit
		fruit_pose = self.get_fruit_pose(centroid, raise_offset)
		while not rospy.is_shutdown():
			try:
				plan = self.planner.plan_to_pose(fruit_pose, [])
				raw_input("Press <Enter> to raise the fruit: ")
				if not self.planner.execute_plan(plan):
					raise Exception("Execution failed")
			except Exception as e:
				print e
			else:
				break

		print("Grabbed Fruit: ", fruit.get_fruit_type())



	def move_to_ar(self):
		return


	def move_to_blender(self):
		# Moves fruit to location direcly above blender opening, drops the fruit in

		
		# returns transformation from the base frame to the ar_tag
		blender_trans_stamped = self.find_transform('base', self.blender_ar)
		blender_trans = blender_trans_stamped.transform

		print(blender_trans)

		goal_1 = PoseStamped()
		goal_1.header.frame_id = "base"

		blender_x = 0.0
		blender_y = 0.0
		blender_z = 0.30

		#x, y, and z position of the blender opening (assuming ar_tag is laying on the table in front of the blender)
		# TODO: find blender_x, blender_y, and blender_z to transform from the blender's ar_tag to the blender's physical opening
		goal_1.pose.position.x = blender_trans.translation.x + blender_x
		goal_1.pose.position.y = blender_trans.translation.y + blender_y
		goal_1.pose.position.z = blender_trans.translation.z + blender_z

		goal_1.pose.orientation.x = 0.0
		goal_1.pose.orientation.y = -1
		goal_1.pose.orientation.z = 0.0
		goal_1.pose.orientation.w = 0.0


		self.blender_pose = goal_1



		# Moves gripper back up to a little above the fruit's centroid
		while not rospy.is_shutdown():
			try:

				plan = self.planner.plan_to_pose(goal_1, [])
				# plan = planner.plan_to_pose(goal_2, [])

				raw_input("Press <Enter> to move to the blender opening: ")
				if not self.planner.execute_plan(plan):
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
				self.move_to_blender(self.blender_ar)
			else:
				break



	def get_fruit_pose(self, centroid, offset=0.0):
		# Takes in TransformStamped message type
		# Returns a Pose stamped message type to the location a LITTLE ABOVE the actual fruit centroid

		goal_1 = PoseStamped()
		goal_1.header.frame_id = "base"

		#x, y, and z position
		goal_1.pose.position.x = centroid[0]
		goal_1.pose.position.y = centroid[1] + offset
		goal_1.pose.position.z = centroid[2]

		#Orientation as a quaternion
		goal_1.pose.orientation.x = 0
		goal_1.pose.orientation.y = -1
		goal_1.pose.orientation.z = 0
		goal_1.pose.orientation.w = 0

		return goal_1

	def find_transform(self, start_frame, goal_frame):
		# find_transform returns TransformStamped, transform is contained in a Transform type
		# Transform contains Vector3 translation and Quaternion rotation

		# To access translation: do realsensecamera_to_ar.transform.translation.{x, y, z}
		# To access quaternion: do realsensecamera_to_ar.transform.rotation{x,y,z,w}
		tfBuffer = tf2_ros.Buffer()
		tfListener = tf2_ros.TransformListener(tfBuffer)
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			try:
				trans = tfBuffer.lookup_transform(start_frame, goal_frame, rospy.Time())
				return trans
			except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
				print(e)
				pass
			r.sleep()


	def grab(self, fruit):
		# fruit is a Fruit object that contains the name of the fruit, the pointcloud, and the first principal component
		# Using pc1 and point cloud, get mean point and orientation

		#Get Fruit's pointcloud (relative to camera)
		#want to find mean of PC1 --> threshold in PC2 direction to find band --> find mean of band in PC2 direction --> grasp around that center
		pointcloud = fruit.points
		pc1 = fruit.pc1 #3x1 vector ??
		pc2 = fruit.pc2

		#Step 1: Un-demean PC1 so we can get the absolute points that lie along the PC1 axis
				# To demean PC1: we find the means of the point cloud and add it back to PC1 (??)
				# multiplying pc1 by g?
				# resource we ended up usign: https://stats.stackexchange.com/questions/229092/how-to-reverse-pca-and-reconstruct-original-variables-from-several-principal-com
		eigenvecs = np.transpose(fruit.components)
		x = fruit.points[:,0]
		y = fruit.points[:,1]
		z = fruit.points[:2]
		mean_x = x.mean()
		mean_y = y.mean()
		mean_z = z.mean()
		mean_vec = np.array([mean_x,mean_y,mean_z])


		pc1_reconstruction = pc1 + eigenvecs + mean_vec #assuming this is a set of points projected onto the PC?

		centroid = np.array([sum(pc1_reconstruction[0]) / len(pc1_reconstruction[0]), sum(pc1_reconstruction[1]) / len(pc1_reconstruction[1]), sum(pc1_reconstruction[2]) / len(pc1_reconstruction[2])])

		# At this point, we should have the reconstructed axis in an absolute position
		#Step 2: Find the mean of the fruit in the PC1 direction
		# To find an actual point: do least squares to the middle of the line




	# def main(self):
	# 	fp = FruitPicker(fruit_example, 'ar_marker_3')
	# 	fp.move_to_blender(fp.blender_ar)

	def create_obstacle(self, planner):
		#creates obstacle for the planner, should be the plane of the table (not sure if it is tho lmao, should test with rviz)
		obstacle_pose = PoseStamped()
		obstacle_pose.pose.position.x = 0.5
		obstacle_pose.pose.position.y = 0.0
		obstacle_pose.pose.position.z = 0.0

		# #Orientation as a quaternion
		obstacle_pose.pose.orientation.x = 0.0
		obstacle_pose.pose.orientation.y = 0.0
		obstacle_pose.pose.orientation.z = 0.0
		obstacle_pose.pose.orientation.w = 1.0

		obstacle_pose.header.frame_id = "base"
		planner.add_box_obstacle([0.4, 1.2, 0.1], "box1", obstacle_pose)


	
if __name__ == '__main__':
	master = FruitPicker('ar_marker_3')
	master.move_to_blender(master.blender_pose)

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shellfBuffer.lookup_transform(turtl

# if __name__ == '__main__':
# 	rospy.init_node('grasp_controller', anonymous=True)
# 	try:
# 		gp = GraspPlanner()
# 		gp.grasp(sys.argv[1], sys.argv[2])
# 	except rospy.ROSInterruptException:
# 		pass
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  # rospy.init_node('turtlebot_controller', anonymous=True)