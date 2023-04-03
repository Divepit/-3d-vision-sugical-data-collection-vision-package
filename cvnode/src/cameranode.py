# rospy for the subscriber
import rospy
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

# Instantiate CvBridge
bridge = CvBridge()

def read_config_file(config_file_path):
    if not os.path.exists(config_file_path):
        raise RuntimeError("Config file doesn't exist: " + config_file_path)
    rospy.loginfo("Read config from: " + config_file_path)

    def read_yaml_file(file_path):
        with open(file_path, 'r') as stream:
            data = yaml.safe_load(stream)
        return data
    config = read_yaml_file(config_file_path)
    return config

def image_callback(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        print("Received an image!")

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


def pointcloud_callback(msg):
    try:
        #TODO do something with pointcloud
        print("Pointcloud received!")
    except:
        print("Error in receiving pointcloud")
    return

def depthImage_callback(msg):
    try: 
        cv2_d_img = bridge.imgmsg_to_cv2(msg)

        print(type(cv2_d_img))
        # cv2.imwrite('camera_depth_image.jpeg', cv2_d_img)

        return
    except CvBridgeError as e:
        print(e)        
        print("Error in receiving depth image!")
        return

def camera_info_callback(msg):
    try:
        height = msg.height
        width = msg.width
        K_intrinsic = msg.K
        return
    except:
        print("Error in receiving intrinsics!")
        return
        

def main():
    # Read config file
    config_path = rospy.get_param("configFile")
    config = read_config_file(config_file_path=config_path)

    cameraNode = rospy.init_node('image_listener')
    # Define your image topic
    image_topic = config["image_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    depthimage_topic = config["depthImage_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
    pointcloud_topic = config["pointCloud_topic_name"] # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html
    cameraInfoTopic = config["cameraInfo_topic_name"] # http://docs.ros.org/en/api/sensor_msgs/html/msg/CameraInfo.html

    # Set up your subscriber and define its callback
    rospy.Subscriber(image_topic, Image, image_callback)
    rospy.Subscriber(pointcloud_topic, PointCloud2, pointcloud_callback)
    rospy.Subscriber(depthimage_topic, Image, depthImage_callback)
    rospy.Subscriber(cameraInfoTopic, CameraInfo, camera_info_callback)

    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()