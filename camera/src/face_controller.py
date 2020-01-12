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

        # initializing camera
        self.face_cascade_name = "/home/kevin/catkin_ws/src/camera/src/haarcascade_frontalface_alt.xml"
        self.eyes_cascade_name = "/home/kevin/catkin_ws/src/camera/src/haarcascade_eye_tree_eyeglasses.xml"
        self.face_cascade = cv.CascadeClassifier()
        self.eyes_cascade = cv.CascadeClassifier()
        self.camera_device = 2
        self.cap = cv.VideoCapture(self.camera_device, cv.CAP_V4L)
        self.check_camera()
        _, frame = self.cap.read()
        self.image_height, self.image_width = frame.shape[:2]

        self.ros_pub_servo_array = rospy.Publisher("/cmd_servo_array", ServoMotorArray, queue_size=1)

        self.bridge = CvBridge()
        self.ros_pub_image = rospy.Publisher("camera/image", Image, queue_size=1)
        rospy.loginfo("[FACE] initalized publisher")

        self.servo_ids = []
        self.servo_angles = []

        self.ros_sub_servo_array = rospy.Subscriber("/low_level_ctrl/servo_array", ServoMotorArray, self.store_servo_state)

        self.servo_array_msg = ServoMotorArray()
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.error_x_last = 0
        self.error_x_sum = 0
        self.error_y_last = 0
        self.error_y_sum = 0

        rospy.loginfo("[FACE] node initialization finished")

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
        kp_x = 0.03
        ki_x = 0.001
        kd_x = 0.01
        kp_y = 0.03
        ki_y = 0.005
        kd_y = 0.01
        if abs(error_x) <= 10:
            d_error_x = 0.0
            self.error_x_sum = 0
            error_x = 0
        else:
            d_error_x = error_x - self.error_x_last
            self.error_x_sum += error_x
        if abs(error_y) <= 20:
            d_error_y = 0.0
            self.error_y_sum = 0
            error_y = 0
        else:
            d_error_y = error_y - self.error_y_last
            self.error_y_sum += error_y


        delta_angle_yaw = -1 * kp_x * error_x + \
                          kd_x * d_error_x
        delta_angle_pitch = -1 * kp_y * error_y + \
                            kd_y * d_error_y

        servo_delta_angle = [delta_angle_yaw, delta_angle_pitch]
        print(servo_delta_angle)

        self.publish_servo_message(servo_delta_angle)
        self.error_x_last = error_x
        self.error_y_last = error_y

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
        cv.destroyAllWindows()
        rospy.loginfo("[FACE] all done!")


if __name__ == '__main__':

    face_controller = FaceController()
    try:
        face_controller.run()
    except rospy.ROSInterruptException:
        pass
