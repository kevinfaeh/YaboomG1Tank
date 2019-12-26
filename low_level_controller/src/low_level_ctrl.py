#!/usr/bin/python
# coding=utf-8

"""
This file holds the low level controller for the tank.
"""
import rospy
import time

from servo_motor import ServoMotor
from dc_motor import DC_Motor
from low_level_controller.msg import DC_Motor_msg
from low_level_controller.msg import DC_MotorArray
from low_level_controller.msg import ServoMotor_msg
from low_level_controller.msg import ServoMotorArray


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
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.dc_motor_array_msg = DC_MotorArray()
        self.dc_motor_array_msg.motors = []
        for i in range(2):
            self.dc_motor_array_msg.motors.append(DC_Motor_msg())

        self.ros_pub_servo_array = rospy.Publisher("/" + self.node_name + "/servo_array", ServoMotorArray, queue_size=1)
        self.ros_pub_dc_motor_array = rospy.Publisher("/" + self.node_name + "/dc_motor_array", DC_MotorArray, queue_size=1)
        rospy.loginfo("initalized publisher")

        self.ros_sub_servo_array = rospy.Subscriber("/cmd_servo_array", ServoMotorArray, self.set_servo_from_command)

        self.ros_sub_dc_motor_array = rospy.Subscriber("/cmd_dc_motor_array", DC_MotorArray, self.set_dc_motor_from_command)
        rospy.loginfo("initalized subscriber")

        self.timeout = 5
        self.last_command_received = time.time()
        rospy.loginfo("node initialization finished")

    def set_servo_from_command(self, message):
        """
        This function sets the servo motors from the subscribed command
        :param message: the message of servo arrays
        :type message ServoMotorArray
        """
        rospy.loginfo(str(message.servos))
        for servo_motor_message in message.servos:
            self.servos[servo_motor_message.id-1].set_angle(servo_motor_message.angle)
        self.publish_servo_message()

    def publish_servo_message(self):
        """
        This function publishes the servo message
        """
        rospy.loginfo("publishing!")
        for i, servo in enumerate(self.servos):
            self.servo_array_msg.servos[i].id = servo.id
            self.servo_array_msg.servos[i].angle = servo.angle
        self.ros_pub_servo_array.publish(self.servo_array_msg)

    def set_dc_motor_from_command(self, message):
        """
        This function sets the servo motors from the subscribed command
        :param message: the message of servo arrays
        :type message ServoMotorArray
        """
        for dc_motor_message in message.motors:
            self.dc_motors[dc_motor_message.id-1].run_motor(dc_motor_message.velocity, dc_motor_message.direction)
        self.publish_dc_motor_message()

    def publish_dc_motor_message(self):
        """
        This function publishes the servo message
        """
        for i, dc_motor in enumerate(self.dc_motors):
            self.dc_motor_array_msg.motors[i].id = dc_motor.id
            self.dc_motor_array_msg.motors[i].velocity = dc_motor.velocity
            self.dc_motor_array_msg.motors[i].direction = dc_motor.direction

        self.ros_pub_dc_motor_array.publish(self.dc_motor_array_msg)

    def run(self):
        """
        This function is to spin the node
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':

    low_level_controller = LowLevelController()
    try:
        low_level_controller.run()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
        pass