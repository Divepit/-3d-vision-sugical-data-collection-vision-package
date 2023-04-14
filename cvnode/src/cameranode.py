# rospy for the subscriber
import rospy

import message_filters
# ROS Image message
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2
# to open yaml
import yaml
import os

# To return a position
import tf2_geometry_msgs




class camera():
    def __init__(self) -> None:
        config_path = rospy.get_param("configFile")
        self.config = self.read_config_file(config_file_path=config_path)

        cameraNode = rospy.init_node(self.config["camera_node_name"])
        # Define your image topic
        image_topic = self.config["image_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        depthimage_topic = self.config["depthImage_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
        pointcloud_topic = self.config["pointCloud_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
        cameraInfoTopic = self.config["cameraInfo_topic_name"] # http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html

        #get camera infos only once
        self.camera_info = rospy.wait_for_message(cameraInfoTopic, CameraInfo, timeout=None)


        image = message_filters.Subscriber(image_topic, Image)
        image_depth = message_filters.Subscriber(depthimage_topic, Image)
        point_cloud = message_filters.Subscriber(pointcloud_topic, PointCloud2)


        # get Synchronize data
        ts = message_filters.ApproximateTimeSynchronizer([image, image_depth, point_cloud], queue_size=10, slop=0.5)
        ts.registerCallback(self.get_syncronous_data)

        # # Set up your subscriber and define its callback
        # rospy.Subscriber(image_topic, Image, self.image_callback)
        # rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloud_callback)
        # rospy.Subscriber(depthimage_topic, Image, self.depthImage_callback)

        # Spin until ctrl + c
        rospy.spin()
    

    def get_syncronous_data(self, image, depth_image, point_cloud):
        self.image_callback(image)
        self.depthImage_callback(depth_image)
        self.pointcloud_callback(point_cloud)

    


    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
            # print("Received an image!")

            # Do something with image

            # Publish result
            return cv2_img
            

        except CvBridgeError as e:
            print("Error in receiving rgb image!")
            print(e)
        else:
            # Save your OpenCV2 image as a jpeg
            print('else: Exception')
            cv2.imwrite('camera_image.jpeg', cv2_img)
            return

    def pointcloud_callback(self, msg):
        # try:
        #     #TODO do something with pointcloud
        #     print("Pointcloud received!")
        # except:
        #     print("Error in receiving pointcloud")
        return

    def depthImage_callback(self, msg):
        try: 
            cv2_d_img = bridge.imgmsg_to_cv2(msg)

            # print(type(cv2_d_img))
            # cv2.imwrite('camera_depth_image.jpeg', cv2_d_img)

            return
        except CvBridgeError as e:
            print(e)        
            print("Error in receiving depth image!")
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
        


if __name__ == '__main__':
    # Instantiate CvBridge
    bridge = CvBridge()

    #start camera node
    camera_class = camera()