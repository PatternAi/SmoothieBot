import numpy as np

from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt


class FruitClassifier(object):
	"""docstring for ClassName
	 1. Initialize FruitClassifier with pointcloud that contains points 
	 	of the table and fruits in the reference frame of the table_ar_tag
	 2. Filter out table to get pointcloud with only fruit clusters
	 3. Run dbscan on fruit clusters and return array of point, assign this to self.fruits

	"""

	def __init__(self, pointcloud):
		self.pointcloud = self.filter_table(pointcloud) # returns pointcloud containing only fruit clusters in the table_ar_tag reference frame
		self.point_dictionary = self.label_pointcloud()
		self.fruits = self.classify_all_fruits(self.point_dictionary)

	def filter_table(self, cloud, offset=0.005):
		x = cloud[:,0]
		y = cloud[:,1]
		z = cloud[:,2]
		r = cloud[:,3]
		g = cloud[:,4]
		b = cloud[:,5]

		z_med = np.median(z)
		z_mask = z > z_med + offset

		mx = np.asarray(x[np.nonzero(z_mask)])
		my = np.asarray(y[np.nonzero(z_mask)])
		mz = np.asarray(z[np.nonzero(z_mask)])
		mr = np.asarray(r[np.nonzero(z_mask)])
		mg = np.asarray(g[np.nonzero(z_mask)])
		mb = np.asarray(b[np.nonzero(z_mask)])

		thresh_xyz = np.asarray(list(zip(mx,my,mz)))
		thresh_rgb = np.asarray(list(zip(mr,mg,mb)))
		thresh_pointcloud = np.hstack((thresh_xyz,thresh_rgb))
		return thresh_pointcloud


	def label_pointcloud(self):
		""" segments point cloud into different point groups using DBSCAN """
		db = DBSCAN(eps=.04, min_samples=300)
		db.fit(self.pointcloud_to_3D(self.pointcloud))
		# core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
		# core_samples_mask[db.core_sample_indices_] = True
		labels = db.labels_
		class_dict = {} # maps: {cluster class => [list of cluster points]}

		for i,label in enumerate(labels):
			if label == -1:
				continue
			if label not in class_dict:
				class_dict[label] = []
			class_dict[label].append(list(self.pointcloud[i]))

		pc = np.array(class_dict[3])
		# x = pc[:,0]
		# y = pc[:,1]
		# z = pc[:,2]
		# r = pc[:,3]
		# g = pc[:,4]
		# b = pc[:,5]

		return class_dict


	def classify_all_fruits(self, point_dict):
		"""point_dictionary: {cluster_class -> fruit point cloud}"""
		classified_fruits = []
		for fruit_points in point_dict.values():

			fruit_type = self.classify_fruit(fruit_points)

			centered_fruit_points = StandardScaler().fit_transform(fruit_points)
			points_3D  = self.pointcloud_to_3D(centered_fruit_points)
			pca = PCA(n_components=3)
			pca.fit(points_3D)
			fruit_centroid = self.find_pointcloud_centroid(fruit_points)
			new_fruit = Fruit(fruit_points, fruit_type, pca.components_, fruit_centroid) 
			classified_fruits.append(new_fruit)

		# print('Classified fruit',classified_fruit)
		return classified_fruits

	def classify_fruit(self, fruit_points):
		"""takes in (K,7) pointcloud that represents a fruit
			return class of fruit
		"""
		# Run PCA to classify as spherical fruit or banana-like fruit
		points_2D = self.pointcloud_to_2D(fruit_points)
		centered_fruit_pts = StandardScaler().fit_transform(points_2D)
		pca = PCA(n_components=2)
		pca.fit(centered_fruit_pts)
		sv1_ratio = pca.explained_variance_ratio_[0]

		print(sv1_ratio)

		# print('Singular values:', sv1_ratio)
		if sv1_ratio > 0.75:
			return 'banana'
		else:
			return self.rgb_classify(fruit_points)


	def find_pointcloud_centroid(self, pointcloud):
		# Inputs: Thresholded NX6 pointcloud (in the frame of the camera_depth_optical_frame)
		# Uses PCA1 and PCA2 to find the centroid
		# Outputs: (x,y) centroid (in the frame of camera_depth_optical_frame)    pointcloud = pointcloud[:,:3] #takes out the RGB values
		x = pointcloud[:,0]
		y = pointcloud[:,1]
		df = pd.DataFrame(np.vstack([x,y]).T)    #We'll need the mean to add back later
		x_mean = np.mean(df[0])
		y_mean = np.mean(df[1])    
		df["x_demeaned"] = df[0] - np.mean(df[0])
		df["y_demeaned"] = df[1] - np.mean(df[1])
		centered = df[["x_demeaned", "y_demeaned"]]  
		u, s, vt = np.linalg.svd(centered, full_matrices = False)
		pcs = np.dot(centered, vt.T)    #pcs
		df["pc1"] = pcs["x_demeaned"]
		df["pc2"] = pcs["y_demeaned"]    
		max_pc1 = df["pc1"].max()
		min_pc1 = df["pc1"].min()
		center1 = (max_pc1 + min_pc1)/2
		range_pc1 = max_pc1-min_pc1    
		max_pc1_roi = center1 + range_pc1/10 
		min_pc1_roi = center1 - range_pc1/10 
		ROI = df[(df["pc1"] >= min_pc1_roi) & (df["pc1"] <= max_pc1_roi)]    
		max_pc2 = ROI["pc2"].max()
		min_pc2 = ROI["pc2"].min()
		center2 = (max_pc2 + min_pc2)/2    
		pc_centers = [center1, center2]
		center = np.dot(pc_centers, vt)
		center[0] += x_mean
		center[1] += y_mean
		return center

	def rgb_classify(self, fruit_points):
		""" takes in a (K,7) point cloud that 
			represents a spherical fruit, returns the fruit type string"""

		rgb = self.pointcloud_to_rgb(fruit_points)
		rgb_mean = np.mean(rgb, axis=0)
		fruit_rgb = {"apple": (80, 20, 10), 
					 "lime" : (15,104,15),
					 "orange" : (200,80,20), 
					 "pear" : (185,120,5)}
		manhattan = lambda x,y : abs(x[0] - y[0]) + abs(x[1] - y[1]) + abs(x[2] - y[2]) 
		distances = {k: manhattan(v, rgb_mean) for k, v in fruit_rgb.items()}
		fruit_name = min(distances, key=distances.get)

		return fruit_name


	########################################################################
	#########			    HELPER FUNCTIONS 					   #########
	########################################################################

	def pointcloud_to_2D(self, pointcloud):
		"""change (N,7) pointcloud points to (N,2) points"""
		pointcloud_2d = np.array(pointcloud)[:,:2]
		# print(pointcloud_2d)
		return pointcloud_2d

	def pointcloud_to_3D(self, pointcloud):
		"""change (N,7) pointcloud points to (N,3) points"""
		pointcloud_3d = np.array(pointcloud)[:,:3]
		return pointcloud_3d

	def pointcloud_to_rgb(self,pointcloud):
		"""change (N,7) pointcloud points to (N,3) points, just the rgb color data"""
		pointcloud_rgb = np.array(pointcloud)[:,3:6]
		return pointcloud_rgb

	def get_fruits(self):
		"""returns the list of available fruit"""
		return self.fruits




class Fruit(object):
	"""Fruit container """

	def __init__(self, points, fruit_type, pca_all, centroid):
		self.points = points
		self.fruit_type = fruit_type
		self.pc1 = pca_all[0]
		self.pc2 = pca_all[1]
		self.pc3 = pca_all[2]
		self.centroid = centroid

		#transformed centroid 
		self.cent_trans = []
		self.pc1_trans = []
		self.pc2_trans = []
		self.pc3_trans = []

	def get_points(self):
		"""return points of fruit"""
		return self.points

	def get_fruit_type(self):
		"""return type of fruit"""
		return self.fruit_type

	def get_pc1(self):
		"""returns first principle component of fruit's point cloud"""
		return self.pc1

	def get_pc2(self):
		"""returns second principle component of fruit's point cloud"""
		return self.pc2

	def get_pc3(self):
		"""returns third principle component of fruit's point cloud"""
		return self.pc3

	def get_centroid(self):
		"""returns centroid of fruit"""
		return self.get_centroid

	def transformAttributes(cent_trans, pc1_trans, pc2_trans, pc3_trans):
		self.cent_trans = cent_trans
		self.pc1_trans = pc1_trans
		self.pc2_trans = pc2_trans
		self.pc3_trans = pc3_trans

	def get_cent_trans(self):
		"""returns centroid in the base frame of the robot"""
		return self.cent_trans

	def get_pc1_trans(self):
		"""returns pc1 in the base frame of the robot"""
		return self.pc1_trans

	def get_pc2_trans(self):
		"""returns pc2 in the base frame of the robot"""
		return self.pc2_trans

	def get_pc3_trans(self):
		"""returns pc1 in the base frame of the robot"""
		return self.pc3_trans

	def __repr__(self):
		"""fruit type print out"""
		return self.fruit_type


def main():

	data = np.load('thresh_cloud.npy')
	x = data[:,0]
	y = data[:,1]
	z = data[:,2]

	# plt.scatter(x,y,z)
	# plt.show()
	

	# data1 = np.random.randint(low=60,high=70, size=(50, 7))

	# data1[:,3:6] = [255,0,0]

	# data2 = np.random.randint(low=50,high=60, size=(50, 7))

	# data2[:,3:6] = [0,255,0]

	# data3 = np.random.randint(low=40,high=50, size=(50, 7))

	# data3[:,3:6] = [0,255,0]

	# data4 = np.random.randint(low=0,high=10, size=(50, 7))

	# data4[:,3:6] = [0,255,0]

	# data = np.vstack([data1,data2,data3,data4])

	fruit_classifier_obj = FruitClassifier(data)

	# twoD = fruit_classifier_obj.pointcloud_to_2D(fruit_classifier_obj.pointcloud)
	# plt.scatter(twoD[:,0], twoD[:,1])
	# plt.show()
	fruit_classifier_obj.dbscan()

	# fruits_expected = ['apple', 'banana', 'orange', 'lime']


	

	for i, fruit in enumerate(fruit_classifier_obj.fruits):
		# print("Expected: "+fruits_expected[i])
		print("Predicted: "+ str(fruit))
		print(fruit.get_pc1())
		print()


if __name__ == '__main__':
	main()