#!/usr/bin/env python
"""
This file is used to give commands to the servos, where a face is recognized.
"""
import cv2 as cv
import rospy

from sensor_msgs.msg import Image
from input_controller.msg import ServoMotor_msg
from input_controller.msg import ServoMotorArray
from cv_bridge import CvBridge


class PIDController:
    """
    This class holds the data of a PID controller.
    """
    def __init__(self, kp, ki, kd):
        """
        This function initializes the PID controller object
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_last = 0
        self.error_sum = 0
        self.delta_error = 0


class FaceController:
    """
    This class is used to control the robot with game pad Input.
    """

    def __init__(self):
        """
        This function initializes gamepad controller.
        """
        self.node_name = "face_ctrl"
        rospy.init_node(self.node_name)

        rospy.loginfo("[FACE] initializing controller")

        self.ros_param_data = self.read_ros_parameters()

        # initializing camera
        self.face_cascade_name = self.ros_param_data["frontal_face_xml_path"]
        self.eyes_cascade_name = self.ros_param_data["eye_xml_path"]
        self.face_cascade = cv.CascadeClassifier()
        self.eyes_cascade = cv.CascadeClassifier()
        self.camera_device = 2
        self.cap = cv.VideoCapture(self.camera_device, cv.CAP_V4L)
        if self.cap is None or not self.cap.isOpened():
            rospy.logerr("[FACE] Could not connect to the camera!")
        self.check_camera()
        _, frame = self.cap.read()
        self.image_height, self.image_width = frame.shape[:2]

        # initialize publisher
        self.ros_pub_servo_array = rospy.Publisher("/cmd_servo_array", ServoMotorArray, queue_size=1)
        self.bridge = CvBridge()
        self.ros_pub_image = rospy.Publisher("camera/image", Image, queue_size=1)
        rospy.loginfo("[FACE] initalized publisher")

        # initialize subscriber
        self.servo_ids = []
        self.servo_angles = []
        self.servo_array_msg = ServoMotorArray()
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.ros_sub_servo_array = rospy.Subscriber("/low_level_ctrl/servo_array", ServoMotorArray, self.store_servo_state)
        rospy.loginfo("[FACE] initialized subscriber")

        # initialize the controllers
        param_pid_yaw = self.ros_param_data["yaw_controller_gains"]
        self.pid_controller_yaw = PIDController(param_pid_yaw["kp"], param_pid_yaw["ki"], param_pid_yaw["kd"])
        param_pid_pitch = self.ros_param_data["pitch_controller_gains"]
        self.pid_controller_pitch = PIDController(param_pid_pitch["kp"], param_pid_pitch["ki"], param_pid_pitch["kd"])

        rospy.loginfo("[FACE] node initialization finished")

    @staticmethod
    def read_ros_parameters():
        """
        This function reads the parameters from the ros parameter server.
        :return: the dictionary with all parameters
        :rtype: dict
        """
        ros_param_data = dict()
        ros_param_data["camera_matrix"] = rospy.get_param("/camera_parameters/matrix")
        ros_param_data["distortion"] = rospy.get_param("/camera_parameters/distortion")
        ros_param_data["frontal_face_xml_path"] = rospy.get_param("/opencv_detector/frontal_face_xml_path")
        ros_param_data["eye_xml_path"] = rospy.get_param("opencv_detector/eye_xml_path")
        ros_param_data["yaw_controller_gains"] = rospy.get_param("opencv_detector/yaw_controller")
        ros_param_data["pitch_controller_gains"] = rospy.get_param("opencv_detector/pitch_controller")

        return ros_param_data

    def check_camera(self):
        """
        This function checks if the camera works properly.
        """
        # -- 2. Read the video stream
        if not self.cap.isOpened:
            rospy.logerr("[FACE] Error opening video capture")
        if not self.face_cascade.load(self.face_cascade_name):
            rospy.logerr("[FACE] Error loading face cascade")
        if not self.eyes_cascade.load(self.eyes_cascade_name):
            rospy.logerr("[FACE] Error loading eye cascade")

    def store_servo_state(self, message):
        """
        This function stores the actual servo state to update it with the controller
        :param message: the servo motor array message
        :type message: ServoMotorArray
        """
        id_data = []
        angle_data = []
        for servo_motor_message in message.servos:
            id_data.append(servo_motor_message.id)
            angle_data.append(servo_motor_message.angle)
        self.servo_ids = id_data
        self.servo_angles = angle_data

    def publish_servo_message(self, value):
        """
        This function publishes the servo message with the update for the index and angle
        :param value: the value to change the current angle
        :type value: list
        """
        if len(self.servo_ids) > 0:
            self.servo_array_msg.servos[0].id = self.servo_ids[0]
            self.servo_array_msg.servos[0].angle = self.servo_angles[0]

            self.servo_array_msg.servos[1].id = self.servo_ids[1]
            self.servo_array_msg.servos[1].angle = self.servo_angles[1] + value[0]

            self.servo_array_msg.servos[2].id = self.servo_ids[2]
            self.servo_array_msg.servos[2].angle = self.servo_angles[2] + value[1]

            self.ros_pub_servo_array.publish(self.servo_array_msg)

    def detect_and_display(self, frame):
        """
        This function runs adaboost to detect the face.
        :param frame: the frame from the camera
        :type frame: numpy array
        :return: the pixel deviation in x and y direction of the recognized face
        :rtype: float, float
        """
        frame_gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        frame_gray = cv.equalizeHist(frame_gray)
        # -- Detect faces
        faces = self.face_cascade.detectMultiScale(frame_gray)
        center = (self.image_width / 2.0, self.image_height / 2.0)
        for (x, y, w, h) in faces:
            center = (x + w // 2, y + h // 2)
            frame_gray = cv.ellipse(frame_gray, center, (w // 2, h // 2), 0, 0, 360, (255, 0, 255), 4)
            # faceROI = frame_gray[y:y + h, x:x + w]
            # # -- In each face, detect eyes
            # eyes = eyes_cascade.detectMultiScale(faceROI)
            # for (x2, y2, w2, h2) in eyes:
            #     eye_center = (x + x2 + w2 // 2, y + y2 + h2 // 2)
            #     radius = int(round((w2 + h2) * 0.25))
            #     frame = cv.circle(frame, eye_center, radius, (255, 0, 0), 4)
            break
        ros_image = self.bridge.cv2_to_imgmsg(frame_gray, "8UC1")
        self.ros_pub_image.publish(ros_image)
        error_x = center[0] - self.image_width / 2.0
        error_y = center[1] - self.image_height / 2.0

        return error_x, error_y

    def servo_controller(self, error_x, error_y):
        """
        This function uses the error of the face position w.r.t. to the center of the image
        to calcualte the command for the servo_motors
        :param error_x: error in x direction
        :type error_x: float
        :param error_y: error in y direction
        :type error_y: float
        """
        print(error_x)
        print(error_y)
        if abs(error_x) <= 10:
            self.pid_controller_yaw.delta_error = 0.0
            self.pid_controller_yaw.error_sum = 0.0
            error_x = 0
        else:
            self.pid_controller_yaw.delta_error = error_x - self.pid_controller_yaw.error_last
            self.pid_controller_yaw.error_sum += error_x
        if abs(error_y) <= 20:
            self.pid_controller_pitch.delta_error = 0.0
            self.pid_controller_pitch.error_sum = 0.0
            error_y = 0
        else:
            self.pid_controller_pitch.delta_error = error_x - self.pid_controller_pitch.error_last
            self.pid_controller_pitch.error_sum += error_x

        input_delta_angle_yaw = -1 * self.pid_controller_yaw.kp * error_x + self.pid_controller_yaw.kd * self.pid_controller_yaw.delta_error

        input_delta_angle_pitch = -1 * self.pid_controller_pitch.kp * error_x + self.pid_controller_pitch.kd * self.pid_controller_pitch.delta_error

        servo_delta_angle = [input_delta_angle_yaw, input_delta_angle_pitch]
        print(servo_delta_angle)

        self.publish_servo_message(servo_delta_angle)
        self.pid_controller_yaw.error_last = error_x
        self.pid_controller_pitch.error_last = error_y

    def detect_face(self):
        """
        This function detects the face
        :return:
        """
        ret, frame = self.cap.read()
        if ret:
            error_x, error_y = self.detect_and_display(frame)
            self.servo_controller(error_x, error_y)

    def run(self):
        """
        This function is used to run the node
        """
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            self.detect_face()
            rate.sleep()
        # clean up while shutdown
        self.cap.release()
        rospy.loginfo("[FACE] all done!")


if __name__ == '__main__':

    face_controller = FaceController()
    try:
        face_controller.run()
    except rospy.ROSInterruptException:
        pass
