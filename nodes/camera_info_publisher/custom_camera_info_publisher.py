#!/usr/bin/env python3

"""
Subscribe to CameraInfo topic replace the message content with new CameraInfo
loaded from a given yaml file, keep the existing timestamp and republish to 
another CameraInfo topic.
"""

import rospy
import yaml
from sensor_msgs.msg import CameraInfo


class CamInfoRepublisher:
    def __init__(self):

        self.cam_info_in = rospy.get_param("~original_cam_info_topic", '/interfacea/link2/camera_info')
        self.cam_info_out = rospy.get_param("~publish_cam_info_to", '/camera/camera_info')
        self.cam_info_yaml_file = rospy.get_param("~cam_info_yaml_file", '')

        self.cam_info_msg = self.read_cam_info_from_file(self.cam_info_yaml_file) 

        self.cam_info_sub = rospy.Subscriber(self.cam_info_in, CameraInfo, self.cam_info_callback)
        self.cam_info_pub = rospy.Publisher(self.cam_info_out, CameraInfo, queue_size=10)
        
    def read_cam_info_from_file(self, yaml_fname):
        
        """
        Contents of this function from:
        https://gist.github.com/rossbar/ebb282c3b73c41c1404123de6cea4771
        ---

        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.
        
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """

        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle)
        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def cam_info_callback(self, msg):

        # store original timestamp
        stamp = msg.header.stamp
        # replace the msg with one from cam_info_yaml_file
        msg = self.cam_info_msg
        #restore the original timestamp in the message
        msg.header.stamp = stamp

        self.cam_info_pub.publish(msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cam_info_republisher')
    node = CamInfoRepublisher()
    node.run()
