# rospy for the subscriber
import rospy
import rospkg
from datetime import datetime

import tf
from tf2_msgs.msg import TFMessage
import tf2_ros
import message_filters
# ROS Image message
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

import sensor_msgs.point_cloud2 as pc2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# to open yaml
import yaml
import os
import numpy as np

# To return a position
import tf2_geometry_msgs

class camera():
    def __init__(self) -> None:

        # Toggle to enable / disable saving images
        self.recordFrames = False
        if self.recordFrames == True:
            ## Required to store results in general
            # get the current timestamp
            now = datetime.now()
            timestamp = now.strftime('%Y-%m-%d-%H-%M-%S')

            self.origPath = os.getcwd()
        
            self.pkg_path = rospkg.RosPack().get_path('cvnode')
            self.resultPath = 'results/' + timestamp + '/'

            self.counter_DepthImage = 0
            self.pathDepth = self.resultPath + 'depth_Images/'
            
            os.chdir(self.pkg_path)
            if not os.path.exists(os.path.dirname(self.pathDepth)):
                os.makedirs(os.path.dirname(self.pathDepth))
            os.chdir(self.origPath)
            ###

            # Required to store RGB images
            ###
            self.counter_RGB = 0
            self.pathRGB = self.resultPath + 'rgb_Images/'

            os.chdir(self.pkg_path)
            if not os.path.exists(os.path.dirname(self.pathRGB)):
                os.makedirs(os.path.dirname(self.pathRGB))
            os.chdir(self.origPath)
            ###

        config_path = rospy.get_param("configFile")
        self.config = self.read_config_file(config_file_path=config_path)

        # Initialize Target position
        self.targetPosition = np.zeros((3,1), dtype=np.float)

        # Initialize Camera Pose
        self.CamPosition = np.zeros((3,1), dtype=np.float)
        self.CamOrient = np.zeros((4,1), dtype=np.float)
        ###
        # Initialize Pose of camera w.r.t camera frame to create transform
        self.Campose_stamped = PoseStamped()
        self.Campose_stamped.header.frame_id = 'world'
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

        rospy.init_node(self.config["camera_node_name"])

        # Define your image topic
        image_topic = self.config["image_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        depthimage_topic = self.config["depthImage_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        pointcloud_topic = self.config["pointCloud_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        cameraInfoTopic = self.config["cameraInfo_topic_name"] # http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html

        # Get name of target position topic and camera position
        targetTopic = self.config["coordinates_of_target"]
        self.cameraFrameName = self.config["cameraPoseTF"]

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
                
        rospy.spin()
    

    def get_syncronous_data(self, image, depth_image, point_cloud):
        self.image_callback(image)
        self.depthImage_callback(depth_image)
        self.pointcloud_callback(point_cloud)


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
                self.counter_RGB = self.saveImage(cv2_img, folderName=self.pathRGB, counter=self.counter_RGB)
                
            return cv2_img

    def pointcloud_callback(self, msg):
        return

    def depthImage_callback(self, msg):
        try: 
            cv2_d_img = bridge.imgmsg_to_cv2(msg)


        except CvBridgeError as e:
            print(e)        
            print("Error in receiving depth image!")
            return
        
        else:
            # Save your OpenCV2 image as a jpeg
            if self.recordFrames == True:
                self.counter_DepthImage = self.saveImage(cv2_d_img, folderName=self.pathDepth, counter=self.counter_DepthImage)

            # Get camera position
            transform = self.tf_buffer.lookup_transform('world', self.cameraFrameName, rospy.get_rostime(), rospy.Duration(0.1))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(self.Campose_stamped, transform)
            self.setCameraPose(pose_transformed)
            self.getTargetCameraDistance()
            centers = self.get_obstacle_centers(cv2_d_img)
            
        return
    
    def get_obstacle_centers(self, cv_d_img) -> list:
        #generate depth mask
        mask_threshold = np.nan_to_num(cv_d_img, nan= np.inf)

        
        distance_mask = mask_threshold < self.camTargetDistance * self.depth_threshold
        close_mask = mask_threshold > self.finger_distance_min 

        mask =  close_mask * distance_mask

        mask_threshold[mask] = 1
        mask_threshold[~mask] = 0

        

        mask_threshold = mask_threshold.astype(np.uint8)

        threshold_image = mask_threshold * 255
        threshold_image = cv2.cvtColor(threshold_image, cv2.COLOR_GRAY2RGB)

        centers = []
        contours, _ = cv2.findContours(mask_threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:

            # calculate moments for each contour
            M = cv2.moments(c)
 
            
            # calculate x,y coordinate of center
            if M["m00"] != 0 :
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                centers.append((cX,cY))
                cv2.circle(threshold_image, (cX, cY), 5, (255, 0, 0), -1)
            else:
                #scip value or else devide by zero error
                continue
        

        show_image = False
        if show_image:
            cv2.namedWindow('img', cv2.WINDOW_NORMAL)
            cv2.imshow('img', threshold_image)
            cv2.waitKey(0)
            try:
                cv2.destroyWindow('img')
            except cv2.error:
                print("Window already closed. Ignocv_d_imgring")
        
        return(centers)


    def targetPositionCallback(self, msg):
        self.targetPosition = np.array([msg.x, msg.y, msg.z])
        return
    
    def setCameraPose(self, pose_msg):
        self.CamPosition = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z])
        self.CamOrient = np.array([pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w])
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
    
    def saveImage(self, image, folderName, counter):
        frameName = str(counter).zfill(5) + '.jpeg'

        os.chdir(self.pkg_path)
        tmp = cv2.imwrite(folderName + frameName, image)
        counter += 1
        os.chdir(self.origPath)

        return counter
        


if __name__ == '__main__':
    # Instantiate CvBridge
    bridge = CvBridge()

    #start camera node
    camera_class = camera()