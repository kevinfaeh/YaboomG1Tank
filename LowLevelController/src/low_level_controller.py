#!/usr/bin/python
# coding=utf-8

"""
This file holds the low level controller for the tank.
"""
import rospy

from LowLevelController.src.servo_motor import ServoMotor
from LowLevelController.src.dc_motor import DC_Motor
from LowLevelController.msg import DC_Motor_msg
from LowLevelController.msg import DC_MotorArray
from LowLevelController.msg import ServoMotor_msg
from LowLevelController.msg import ServoMotorArray


class LowLevelController:
    """
    This class holds all functionality for the low-level controller.
    """

    def __init__(self):
        """
        This function initializes the low level controller.
        """

        rospy.loginfo("initializing low level controller")

        self.servo_front = ServoMotor(1, 23, 50, 110)
        self.servo_cam_yaw = ServoMotor(2, 11, 50, 110)
        self.servo_cam_pitch = ServoMotor(3, 9, 50, 110)

        self.servos = [self.servo_front, self.servo_cam_yaw, self.servo_cam_pitch]

        self.dc_left = DC_Motor(1, 20, 21, 16, 2000)
        self.dc_right = DC_Motor(2, 19, 26, 13, 2000)

        self.dc_motors = [self.dc_left, self.dc_right]

        self.node_name = "low_level_ctrl"
        rospy.init_node(self.node_name)

        self.servo_array_msg = ServoMotorArray()
        for i in range(3):
            self.servo_array_msg.append(ServoMotor_msg())

        self.dc_motor_array_msg = DC_MotorArray()
        for i in range(2):
            self.dc_motor_array_msg.append(DC_Motor_msg())

        self.ros_pub_servo_array = rospy.Publisher("/" + self.node_name + "/servo_array", ServoMotorArray, queue_size=1)
        self.ros_pub_dc_motor_array = rospy.Publisher("/" + self.node_name + "/dc_motor_array", DC_MotorArray, queue_size=1)
        rospy.loginfo("initalized publisher")

        self.ros_sub_





