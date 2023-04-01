import rospy
import os
import yaml

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

class cameraNode(object):
    
    def __init__(self):
        # Read config file
        config_path = rospy.get_param("configFile")
        config = read_config_file(config_file_path=config_path)

        #Get Names
        self.nodeName = config["camera_node_name"]
        self.imageTopic = config["image_topic_name"]
        self.depthimageTopic = config["depthImage_topic_name"]
        
        self.K_int, self.w_img, self.h_img = self.receiveCameraInfo()

        self.receiveFormat

        self.image
        self.depthimage


        return
    
    def receiveCameraInfo(self):
        """
        Subscriber callback for camera info. To get camera intrinsic, format, etc.
        """
        return
    
    
    

    
