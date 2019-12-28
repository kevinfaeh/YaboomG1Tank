#!/usr/bin/python
# coding=utf-8

"""
This file holds the low level controller for the tank.
"""
import rospy
import RPi.GPIO as GPIO
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
        self.node_name = "low_level_ctrl"
        rospy.init_node(self.node_name)

        rospy.loginfo("initializing low level controller")

        self.servos, self.dc_motors = self.launch_motors(1)

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
        rospy.loginfo("[LLC] initalized publisher")

        self.ros_sub_servo_array = rospy.Subscriber("/cmd_servo_array", ServoMotorArray, self.set_servo_from_command)

        self.ros_sub_dc_motor_array = rospy.Subscriber("/cmd_dc_motor_array", DC_MotorArray, self.set_dc_motor_from_command)
        rospy.loginfo("[LLC] initalized subscriber")

        rospy.loginfo("[LLC] node initialization finished")

    def launch_motors(self, wait_time):
        """
        This function launches all motors.
        :param wait_time: time to wait between Pin allocation on GPIO (if not waited -> raspberry pi reboots)
        :type wait_time: int
        """

        servo_front = ServoMotor(1, 23, 50, 110)
        time.sleep(wait_time)
        rospy.loginfo("[LLC] launched servo front")
        servo_cam_yaw = ServoMotor(2, 11, 50, 110)
        time.sleep(wait_time)
        rospy.loginfo("[LLC] launched servo yaw")
        servo_cam_pitch = ServoMotor(3, 9, 50, 110)
        time.sleep(wait_time)
        rospy.loginfo("[LLC] launched servo pitch")

        servos = [servo_front, servo_cam_yaw, servo_cam_pitch]

        dc_left = DC_Motor(1, 20, 21, 16, 2000)
        time.sleep(wait_time)
        rospy.loginfo("[LLC] launched dc motor left")
        dc_right = DC_Motor(2, 19, 26, 13, 2000)
        time.sleep(wait_time)
        rospy.loginfo("[LLC] launched dc motor right")
        dc_motors = [dc_left, dc_right]

        rospy.loginfo("[LLC] all motors launched")
        return servos, dc_motors

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
        rospy.loginfo("[LLC] publishing")
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
        # clean up while shutdown
        rospy.loginfo("[LLC] shutting down motors..")
        self.shut_down_motors()
        GPIO.cleanup()
        rospy.loginfo("[LLC] all done!")

    def shut_down_motors(self):
        """
        This function shuts down all motors.
        """
        for motor in self.servos:
            motor.stop_motor()
        for motor in self.dc_motors:
            motor.stop_motor()


if __name__ == '__main__':

    low_level_controller = LowLevelController()
    try:
        low_level_controller.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[LLC] shutting down motors..")
        low_level_controller.shut_down_motors()
        GPIO.cleanup()
        rospy.loginfo("[LLC] all done!")
