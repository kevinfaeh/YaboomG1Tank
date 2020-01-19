#!/usr/bin/env python
"""
This file is used to give commands to the servos, where a face is recognized.
"""
import cv2 as cv
import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class AcquireImage:
    """
    This class is used to control the robot with game pad Input.
    """
    def __init__(self):
        """
        This function initializes gamepad controller.
        """
        self.node_name = "image_acquisition"
        rospy.init_node(self.node_name)

        rospy.loginfo("[ACQUIRE IMAGE] initializing image acquisition node")

        self.ros_param_data = self.read_ros_parameters()
        self.frequency = self.ros_param_data["frequency"]

        # initializing camera
        self.camera_device = self.ros_param_data["camera_device_port"]
        self.cap = cv.VideoCapture(self.camera_device, cv.CAP_V4L)
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("[ACQUIRE IMAGE] Could not connect to the camera!")
        _, frame = self.cap.read()
        self.image_width = self.ros_param_data["image_width"]
        self.image_height = self.ros_param_data["image_height"]

        # initialize publisher
        self.bridge = CvBridge()
        self.ros_pub_image = rospy.Publisher("image_acquisition/image", Image, queue_size=1)
        rospy.loginfo("[ACQUIRE IMAGE] initalized publisher")

        rospy.loginfo("[ACQUIRE IMAGE] node initialization finished")

    @staticmethod
    def read_ros_parameters():
        """
        This function reads the parameters from the ros parameter server.
        :return: the dictionary with all parameters
        :rtype: dict
        """
        ros_param_data = dict()
        ros_param_data["frequency"] = rospy.get_param("/image_acquisition/frequency")
        ros_param_data["image_height"] = rospy.get_param("/image_acquisition/image_height")
        ros_param_data["image_width"] = rospy.get_param("/image_acquisition/image_width")
        ros_param_data["camera_device_port"] = rospy.get_param("/image_acquisition/camera_device_port")
        ros_param_data["camera_matrix"] = rospy.get_param("/image_acquisition/camera_parameters/matrix")
        ros_param_data["distortion"] = rospy.get_param("image_acquisition/camera_parameters/distortion")

        return ros_param_data

    def convert_image_and_publish(self, frame):
        """
        This function runs adaboost to detect the face.
        :param frame: the frame from the camera
        :type frame: numpy array
        """
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        #frame_gray = cv.equalizeHist(frame_gray)
        frame_gray = self.resize_image(frame_gray)
        ros_image = self.bridge.cv2_to_imgmsg(frame_gray, "8UC1")
        self.ros_pub_image.publish(ros_image)

    def resize_image(self, image):
        """
        This function resizes the image
        :param image: the acquired image
        :type image: numpy array
        :return: the resized image
        :rtype numpy array
        """

        dim = (self.image_width, self.image_height)
        resized_image = cv.resize(image, dim, interpolation=cv.INTER_AREA)
        return resized_image

    def acquire_image(self):
        """
        This function detects the face
        :return:
        """
        ret, frame = self.cap.read()
        if ret:
            self.convert_image_and_publish(frame)

    def run(self):
        """
        This function is used to run the node
        """
        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            self.acquire_image()
            rate.sleep()
        # clean up while shutdown
        self.cap.release()
        rospy.loginfo("[ACQUIRE IMAGE] all done!")


if __name__ == '__main__':

    acquire_image = AcquireImage()
    try:
        acquire_image.run()
    except rospy.ROSInterruptException:
        pass
