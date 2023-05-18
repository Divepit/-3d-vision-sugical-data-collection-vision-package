# rospy for the subscriber
import rospy
import rospkg
from datetime import datetime
import math

import tf
from tf2_msgs.msg import TFMessage
import tf2_ros
import message_filters
# ROS Image message
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped, Vector3, PointStamped, TransformStamped, Polygon, Point32
from std_msgs.msg import Bool
from cvnode.msg import Sphere, SphereList

import sensor_msgs.point_cloud2 as pc2
# OpenCV2 for saving an image
import cv2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

#For PCA
from sklearn.decomposition import PCA

# to open yaml
from PIL import Image as pilimage
import yaml
import os
import numpy as np

# To return a position
import tf2_geometry_msgs

class SaveImage():
    def __init__(self, savePath, origPath, pkgName = 'cvnode'):

        # Required to Store depth image
        self.counter = 0
        pkg_path = rospkg.RosPack().get_path(pkgName)

        self.savePath_abs = pkg_path +  '/' + savePath

        self.savePath = savePath
        self.origPath = origPath
        
        os.chdir(pkg_path)
        if not os.path.exists(os.path.dirname(self.savePath)):
            os.makedirs(os.path.dirname(self.savePath))
        os.chdir(self.origPath)

        return
    
    def saveImage(self, img, typeSave = cv2.CV_8U, normalize = False):
        if typeSave == cv2.CV_32F:
            extention = '.exr'
        else:
            extention = '.png'
        frameName = str(self.counter).zfill(6) + extention

        if normalize == True:
            if np.max(img) != 0:
                img = ((img - np.min(img)) / np.max(img) * 255)

        if typeSave == cv2.CV_16U:
            # Convert to PIL Image and save as PNG
            pil_img = pilimage.fromarray(img, mode='I;16')
            os.chdir(self.savePath_abs)
            pil_img.save(frameName)
            self.counter += 1
            os.chdir(self.origPath)
            
        else:
            os.chdir(self.savePath_abs)
            tmp = cv2.imwrite(frameName, img, [typeSave])
            self.counter += 1
            os.chdir(self.origPath)

        return

class camera():
    def __init__(self) -> None:

        # Toggle to enable / disable saving images
        self.recordFrames = False
        if self.recordFrames == True:
            ## Required to store results in general
            # get the current timestamp
            now = datetime.now()
            timestamp = now.strftime('%Y-%m-%d-%H-%M-%S')
            origPath = os.getcwd()
            resultPath = 'results/' + timestamp + '/'

            # Required to Store depth image
            ###
            pathDepth = resultPath + 'depth_Images/'
            self.saveDepth = SaveImage(pathDepth, origPath, pkgName='cvnode')
            ###

            # Required to Store depth image normalized
            ###
            pathDepth_N = resultPath + 'depth_Images_normalized/'
            self.saveDepth_N = SaveImage(pathDepth_N, origPath, pkgName='cvnode')
            ###

            # Required to Store RGB image
            ###
            pathrgb = resultPath + 'rgb_Images/'
            self.saveRGB = SaveImage(pathrgb, origPath, pkgName='cvnode')
            ###

            # Required to Store Masked Depth image
            ###
            pathMaskedD = resultPath + 'masked_Depth_Images/'
            self.saveMasked_D = SaveImage(pathMaskedD, origPath, pkgName='cvnode')
            ###

        config_path = rospy.get_param("configFile")
        self.config = self.read_config_file(config_file_path=config_path)

        # Initialize Target position
        self.targetPosition = np.zeros((3,1), dtype=float)

        # Initialize Camera Pose
        self.CamPosition = np.zeros((3,1), dtype=float)
        self.CamOrient = np.zeros((4,1), dtype=float)
        ###
        # Initialize Pose of camera w.r.t camera frame to create transform
        self.Campose_stamped = PoseStamped()
        self.Campose_stamped.header.frame_id = 'root'
        self.Campose_stamped.pose.position.x = 0.0
        self.Campose_stamped.pose.position.y = 0.0
        self.Campose_stamped.pose.position.z = 0.0
        self.Campose_stamped.pose.orientation.x = 0.0
        self.Campose_stamped.pose.orientation.y = 0.0
        self.Campose_stamped.pose.orientation.z = 0.0
        self.Campose_stamped.pose.orientation.w = 1.0
        ###

        # Initialize Camera - Target distance
        self.camTargetDistance = 0

        # Initialize depth image threshold in percent
        self.depth_threshold = 0.8
        # Min distance from camera
        self.finger_distance_min = 0.1

        #Sphere generation with recursion parameters
        self.max_sphere_radius = 0.07
        self.max_recursions = 10

        rospy.init_node(self.config["camera_node_name"])

        # Define your image topic
        image_topic = self.config["image_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        depthimage_topic = self.config["depthImage_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        pointcloud_topic = self.config["pointCloud_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        cameraInfoTopic = self.config["cameraInfo_topic_name"] # http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html
        lineOfSightTopic = self.config["lineofsight_topic_name"] 

        # Get name of target position topic and camera position
        targetTopic = self.config["coordinates_of_target"]
        self.cameraFrameName = self.config["cameraPoseTF"]

        # Get name of publishing topic
        maskedDepth_topic = self.config["maskedDepth_topic"]
        obstacleCenter_topic = self.config["obstacle_list_topic"]

        # get camera infos once to initialize
        self.camera_info = rospy.wait_for_message(cameraInfoTopic, CameraInfo, timeout=None)

        # Subscribe to image topics
        image = message_filters.Subscriber(image_topic, Image)
        image_depth = message_filters.Subscriber(depthimage_topic, Image)
        point_cloud = message_filters.Subscriber(pointcloud_topic, PointCloud2)
        # get Synchronize data
        ts = message_filters.ApproximateTimeSynchronizer([image, image_depth, point_cloud], queue_size=10, slop=0.5)
        ts.registerCallback(self.get_syncronous_data)

        # Subscribe to target position
        rospy.Subscriber(targetTopic, Point, self.targetPositionCallback)

        # Create tf listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create publisher for masked depth image
        self.masked_d_img_pub = rospy.Publisher(maskedDepth_topic, Image, queue_size=10)
        self.obstacleCenter_pub = rospy.Publisher(obstacleCenter_topic, SphereList, queue_size=10)

        # Create publisher for line of sight condition
        self.line_of_sight_pub = rospy.Publisher(lineOfSightTopic, Bool, queue_size=10) 
                
        rospy.spin()
    

    def get_syncronous_data(self, image, depth_image, point_cloud):
        #main callback function
        self.get_world_data()
        self.image_callback(image)
        self.depthImage_callback(depth_image)
        self.pointcloud_callback(point_cloud)


    def get_world_data(self):
        # Get camera position
        self.transform_camera_to_world = self.tf_buffer.lookup_transform('root', self.cameraFrameName, rospy.get_rostime(), rospy.Duration(0.1))
        self.transform_wolrd_to_camera = self.tf_buffer.lookup_transform(self.cameraFrameName, 'root', rospy.get_rostime(), rospy.Duration(0.1))

        pose_transformed = tf2_geometry_msgs.do_transform_pose(self.Campose_stamped, self.transform_camera_to_world)

        self.setCameraPose(pose_transformed)
        self.getTargetCameraDistance()


    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            
        except CvBridgeError as e:
            print("Error in receiving rgb image!")
            print(e)

        else:

            # Save your OpenCV2 image as a jpeg
            if self.recordFrames == True:
                self.saveRGB.saveImage(cv2_img,typeSave=cv2.CV_8U, normalize=False)
                
            return cv2_img

    def pointcloud_callback(self, msg):
        return

    def depthImage_callback(self, msg):
        try: 
            cv2_d_img = bridge.imgmsg_to_cv2(msg)   # Float32 depth image in m

            ################ 1 channel uint16 ################
            # Convert to 3 channel uint8
            cv2_d_img_mm = cv2_d_img * 1000          # Float32 depth image in mm
            cv2_d_img_mm = np.nan_to_num(cv2_d_img_mm, nan=65535, posinf=65535, neginf=0)
            # Clip values to a 16-bit range
            depth_clipped = np.clip(cv2_d_img_mm, 0, 65535)

            # Convert the depth data to an unsigned 16-bit integer numpy array
            d_img_uint16 = depth_clipped.astype(np.uint16)
            ################ 1 channel uint16 ################

            ################ 3 channel uint8 ################
            # # Convert to 3 channel uint8
            # cv2_d_img_mm = cv2_d_img * 1000          # Float32 depth image in mm
            # h, w = np.shape(cv2_d_img_mm)
            # d_img_uint8 = np.zeros((h,w,3), dtype=np.uint8) # empty uint8 3 channel image

            # # # Encode information using logarithmic scale for each channel
            # # log_scale_1 = np.log2(cv2_d_img_mm + 1) / np.log2(2**16)
            # # log_scale_2 = np.log2(cv2_d_img_mm + 1) / np.log2(2**8)
            # # log_scale_3 = np.log2(cv2_d_img_mm + 1) / np.log2(2**1)

            # # # Convert to uint8 and assign to channels
            # # d_img_uint8[:,:,0] = (log_scale_1 * 255).astype(np.uint8)
            # # d_img_uint8[:,:,1] = (log_scale_2 * 255).astype(np.uint8)
            # # d_img_uint8[:,:,2] = (log_scale_3 * 255).astype(np.uint8)
            ################ 3 channel uint8 ################


        except CvBridgeError as e:
            print(e)        
            print("Error in receiving depth image!")
            return
        
        else:
            # Save your OpenCV2 image as a jpeg
            if self.recordFrames == True:
                self.saveDepth.saveImage(cv2_d_img, cv2.CV_32F)
                self.saveDepth_N.saveImage(cv2_d_img,normalize=True)

            self.target_point = self.project_world_point_onto_camera(self.targetPosition)
            spheres = self.get_obstacle_centers(cv2_d_img)

            
            if len(spheres) != 0:
                self.publishObstacles(spheres)

            # Get masked depth image and publish it
            
            mask = self.get_depth_mask(cv2_d_img, self.finger_distance_min, self.camTargetDistance * self.depth_threshold )
            mask = mask.astype(np.uint16)

            threshold_image = mask * d_img_uint16

            if self.recordFrames == True:
                self.saveMasked_D.saveImage(threshold_image,typeSave=cv2.CV_8U, normalize=True)

            masked_depth_msg = bridge.cv2_to_imgmsg(cvim = threshold_image)
            self.masked_d_img_pub.publish(masked_depth_msg)

            # check and publish line of sight to target
            lineOfSight = self.checkLineOfSight(mask,self.targetPosition)
            boolmessage = Bool()
            boolmessage.data = lineOfSight
            self.line_of_sight_pub.publish(boolmessage)
            
        return
    
    def checkLineOfSight(self, mask, targetPosition_worldFrame, pixelradius = 2):
        cameraFrame_point = self.get_point_in_camera_frame(targetPosition_worldFrame)
        image_coord = self.project_world_point_onto_camera(targetPosition_worldFrame)
        # Initialize Line of sight bool 
        lineOfSight = True

        x_img = image_coord[0].astype(int)
        y_img = image_coord[1].astype(int)

        # check if target is behind camera
        if cameraFrame_point[2] <= 0:
            lineOfSight = False

        # Check if target is outside of image frame
        if self.camera_info.height - pixelradius < y_img or y_img + pixelradius < 0:
            lineOfSight = False
        if self.camera_info.width - pixelradius < x_img or x_img + pixelradius < 0:
            lineOfSight = False

        # Check if mask at target position and pixelradius around it is empty
        if lineOfSight == True:
            targetRegion = mask[y_img-pixelradius:y_img+pixelradius, x_img-pixelradius:x_img+pixelradius ]
            isObstructed = np.any(targetRegion == 1)
            if isObstructed == True:
                lineOfSight = False

        return lineOfSight

    
    def get_depth_mask(self,depth_image, min_distance, max_distance):
        #returns a mask with all 1 for distance values between min and max distance everything else is 0
    
        #replace all nan with np.inf
        mask_threshold = np.nan_to_num(depth_image, nan= np.inf)
        
        max_mask = mask_threshold < max_distance
        min_mask = mask_threshold > min_distance

        mask =  max_mask * min_mask

        mask_threshold[mask] = 1
        mask_threshold[~mask] = 0

        return mask_threshold


    def get_obstacle_centers(self, cv_d_img) -> list:
        #generate depth mask

        mask = self.get_depth_mask(cv_d_img, self.finger_distance_min, self.camTargetDistance * self.depth_threshold )
        

        mask = mask.astype(np.uint8)

        threshold_image = mask * cv_d_img
        threshold_image = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2RGB)

        spheres = []
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i, contour in enumerate(contours):

            # calculate moments for each contour
            filled_mask = np.zeros_like(mask)

            # Draw the contours on the black mask
            cv2.drawContours(
                image=filled_mask,
                contours=[contour],
                contourIdx=0,
                color= 1,
                thickness = -1)
            
            filled_mask = filled_mask.astype(bool)

            recursion_spheres = self.get_spheres_recursion(filled_mask, cv_d_img, 0)
            spheres += recursion_spheres

        debug = False
        if debug and len(spheres) > 0:
            print(f"number of spheres {len(spheres)}")
            
            #print(spheres)
            #sphere_array = np.array(spheres)
            #print(sphere_array)
            radius = [elem[1] for elem in spheres]

            radius_array = np.array(radius)
            max_radius = np.max(radius_array)
            #print(sphera_array)
            #max_value_third_col = np.max(sphera_array[:, 2])
            print("Maximum radius: ", max_radius)
            radius_too_large = np.sum(radius_array > self.max_sphere_radius)
            print(f"The Number of spheres that are larger than {self.max_sphere_radius} m is: {radius_too_large}")



        show_image = False
        if show_image:
            cv2.namedWindow('img', cv2.WINDOW_NORMAL)
            cv2.imshow('img', threshold_image)
            cv2.waitKey(0)
            try:
                cv2.destroyWindow('img')
            except cv2.error:
                print("Window already closed. Ignocv_d_imgring")

        return spheres
    
    def get_spheres_recursion(self, mask, depth_img, recursion_num):
        sphere_list = []
        sphere = self.get_sphere_from_mask(mask, depth_img)
        
        #if the sphere radius is still too large and we didn't exceed our recursion number split and recurse
        if sphere[1] > self.max_sphere_radius and recursion_num < self.max_recursions:
            recursion_num += 1
            mask_1, mask_2 = self.split_mask_using_pca(mask)
            spheres_1 = self.get_spheres_recursion(mask_1, depth_img, recursion_num)
            spheres_2 = self.get_spheres_recursion(mask_2, depth_img, recursion_num)
            sphere_list = spheres_1 + spheres_2
        else:
            sphere_list.append(sphere)

        return sphere_list
        
        
    def split_mask_using_pca(self, mask):
        assert len(mask.shape) == 2, "Input mask should be a 2D binary array"
        
        x_points, y_points = np.where(mask)
        points_of_interest = np.column_stack((x_points, y_points))
        # Extract contour points
        
        contour_points = np.vstack(points_of_interest).squeeze()


        # Perform PCA on the contour points
        pca = PCA(n_components=2)
        pca.fit(contour_points)


        # Project contour points onto the first principal component
        projected_points = pca.transform(contour_points)[:, 0]

        # Find the median value of the projected points
        mean_point = np.mean(projected_points)

        # Separate points based on the median value
        points1 = contour_points[projected_points <= mean_point]
        points2 = contour_points[projected_points > mean_point]

        # Create two empty masks
        mask1 = np.zeros_like(mask)
        mask2 = np.zeros_like(mask)

        # Fill the two masks with separated points
        for pt in points1:
            mask1[pt[0], pt[1]] = 255

        for pt in points2:
            mask2[pt[0], pt[1]] = 255

        # Return the two masks
        return mask1, mask2
    
    def get_sphere_from_mask(self,filled_mask,depth_img):

        # Get x, y coordinates of true values in binary mask
            y_coords, x_coords = np.where(filled_mask)

            # Get depth values at those coordinates
            depth_values = depth_img[y_coords, x_coords]

            # remove nan
            cond = [~np.isnan(i) for i in depth_values]
            x_coords, y_coords, depth_values = x_coords[cond], y_coords[cond], depth_values[cond]
            
            # Combine x, y, depth values into a single numpy array
            point_array = np.column_stack((x_coords, y_coords, depth_values))

            # Get radius and center in pixel
            center, radius = self.getImageCircle(point_array)

            # Get depth of circle
            depth_center = np.min(point_array[:,2])

            point_array_2d = np.array([[center[0] ,center[1] ,depth_center]])

            center_3d, radius_3d = self.getCenter_Radius_fromPixel(point_array_2d,radius)

            center_world = self.get_point_in_world_frame(center_3d)

            sphere = [center_world,radius_3d,0,0]

            return sphere

    
    def getCenter_Radius_fromPixel(self,point_array,radius):

        pts1 = point_array
        pts2 = np.zeros_like(point_array)
        pts2[:,:] = point_array[:,:]  
        pts2[:,0] -= radius

        pts1_3d = self.get3dPoints(pts1) 
        pts2_3d = self.get3dPoints(pts2)

               

        radius = np.linalg.norm(pts1_3d-pts2_3d)

        return pts1_3d[0,:], radius
    
    def getImageCircle(self,point_array):
        x = point_array[:,0]
        y = point_array[:,1]

        x_center = np.mean(x,axis=0)
        y_center = np.mean(y,axis=0)

        x_dist = np.abs(x-x_center)
        y_dist = np.abs(y-y_center)
        dist = x_dist**2 + y_dist**2
        rad_sqrd = np.max(dist)

        return [x_center,y_center], np.sqrt(rad_sqrd)


    def targetPositionCallback(self, msg):
        self.targetPosition = np.array([msg.x, msg.y, msg.z])
        return
    
    def setCameraPose(self, pose_msg):
        self.CamPosition = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.CamOrient = np.array([pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z])
        return
    
    def getTargetCameraDistance(self):
        delta = self.targetPosition - self.CamPosition
        self.camTargetDistance = np.linalg.norm(delta)
        return

    def read_config_file(self, config_file_path):
        if not os.path.exists(config_file_path):
            raise RuntimeError("Config file doesn't exist: " + config_file_path)
        rospy.loginfo("Read config from: " + config_file_path)

        def read_yaml_file(file_path):
            with open(file_path, 'r') as stream:
                data = yaml.safe_load(stream)
            return data
        config = read_yaml_file(config_file_path)
        return config
    
    def get_point_in_camera_frame(self, point) -> np.array:
        # Input:  array [x, y, z] in world frame 
        # Output: array [x, y, z] in camera frame
        return self.transform_point(point, self.transform_wolrd_to_camera)
    
    def get_point_in_world_frame(self, point) -> np.array:
        # Input:  array [x, y, z] in camera frame 
        # Output: array [x, y, z] in world frame
        return self.transform_point(point, self.transform_camera_to_world)
        
    def transform_point(self, point: np.array, transform: TransformStamped) -> np.array:
        # Transform point into new frame using transform


        # create PointStamped
        point_stamped = PointStamped()
        point_stamped.point.x = point[0]
        point_stamped.point.y = point[1]
        point_stamped.point.z = point[2]

        #transform point using the transform
        transformed_point = tf2_geometry_msgs.do_transform_point(point_stamped, transform)

        #change format to array of type [x, y, z]
        new_point = np.array([transformed_point.point.x, transformed_point.point.y, transformed_point.point.z])
        return new_point
        
    def project_world_point_onto_camera(self, point: np.array):
        # Input: Points in the wolrd coordinate frames
        # Output: array(x,y) coordinates on image
        point = self.get_point_in_camera_frame(point)

        K = np.array(self.camera_info.K)
        K = K.reshape((3, 3))

        P_camera, _ = cv2.projectPoints(point.T, np.zeros((3, 1)), np.zeros((3, 1)), K, None)
        image_coordinates = P_camera[0][0]

        return image_coordinates
    

    #TODO delete when colin is not a lil bitch
    def get3dCenters(self,centers,d_img):
        
        if len(centers) == 0:
            return []
        
        centers3d = [None] * len(centers)
        K = self.camera_info.K

        for i in range(len(centers)):
            center = centers[i]

            u, v = center[0], center[1]

            # In camera coordinate frame
            z = d_img[v,u]
            x = (u - K[2]) / K[0] * z
            y = (v - K[5]) / K[4] * z
            
            # Transform in world frame
            center3d_world = self.get_point_in_world_frame(np.array([x,y,z]))
            centers3d[i] = center3d_world

        return centers3d

    def get3dPoints(self, points_2d_depth: np.array):

        K = np.array(self.camera_info.K)
        K = K.reshape((3, 3))
        # Separate the 2D points and depth values
        points_2d = points_2d_depth[:, :2]
        depths = points_2d_depth[:, 2]

        dist_coeffs = np.zeros(5,)


        # Undistort and normalize the image points
        img_points = points_2d.reshape(-1, 1, 2).astype(np.float32)
        normalized_points = cv2.undistortPoints(img_points, K, dist_coeffs)

        # Obtain the 3D points in the camera coordinate system
        points_3d = normalized_points * depths.reshape(-1, 1, 1)
        points_3d = points_3d.reshape(-1, 2)
        points_3d = np.hstack((points_3d, depths.reshape(-1, 1)))

        return points_3d
    
    def calculate_sphere_attributes(self, points):
        center = np.mean(points, axis=0)

        #TODO calculate center in world frame not camera frame
        center_world = self.get_point_in_world_frame(center)

        # Calculate the distances from the center to each point
        distances = np.linalg.norm(points - center, axis=1)

        # Calculate the max distance
        max_distance = np.max(distances)
        radius = max_distance

        # Calculate the standard deviation and variance of the distances
        std_dev = np.std(distances)
        variance = np.var(distances)

        # print("Center:", center)
        # print("Max distance, radius:", max_distance)
        # print("Standard deviation:", std_dev)
        # print("Variance:", variance)
        return [center_world, radius, std_dev, variance]

    
    def publishObstacles(self, spheres):


        obstacle_msg = SphereList()

        for sphere_element in spheres:

            sphere = Sphere()
            sphere.center.x = sphere_element[0][0]
            sphere.center.y = sphere_element[0][1]
            sphere.center.z = sphere_element[0][2]

            sphere.radius = sphere_element[1]
            # sphere.std_dev = sphere_element[2]
            # sphere.variance = sphere_element[3]

            obstacle_msg.spheres.append(sphere)

        self.obstacleCenter_pub.publish(obstacle_msg)
        return


if __name__ == '__main__':
    # Instantiate CvBridge
    bridge = CvBridge()

    #start camera node
    camera_class = camera()