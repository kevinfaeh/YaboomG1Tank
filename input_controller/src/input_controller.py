#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from input_controller.msg import DC_Motor_msg
from input_controller.msg import DC_MotorArray
from input_controller.msg import ServoMotor_msg
from input_controller.msg import ServoMotorArray


class KeybardInput:
    """
    This class is used to control the robot with Keybard Input.
    """

    def __init__(self):
        """
        This function initializes input controller.
        """
        self.node_name = "input_ctrl"
        rospy.init_node(self.node_name)

        rospy.loginfo("[INP] initializing input controller")

        self.ros_pub_servo_array = rospy.Publisher("/cmd_servo_array", ServoMotorArray, queue_size=1)
        self.ros_pub_dc_motor_array = rospy.Publisher("/cmd_dc_motor_array", DC_MotorArray, queue_size=1)
        rospy.loginfo("[INP] initalized publisher")

        self.ros_sub_servo_array = rospy.Subscriber("/cmd_vel", Twist, self.set_motors_from_twist)

        rospy.loginfo("[INP] initalized subscriber")

        self.servo_array_msg = ServoMotorArray()
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.dc_motor_array_msg = DC_MotorArray()
        self.dc_motor_array_msg.motors = []
        for i in range(2):
            self.dc_motor_array_msg.motors.append(DC_Motor_msg())

        rospy.loginfo("[INP] node initialization finished")

    def set_motors_from_twist(self, twist_message):
        """
        This function converst the twist message to a motor command
        :param twist_message: the twist message
        :type twist_message: Twist
        """
        x = twist_message.linear.x
        y = twist_message.linear.y
        if x == 0 and y == 0:
            r_vel = 0
            r_dir = 1
            l_vel = 0
            l_dir = 1
        elif x != 0 and y == 0:
            r_vel = x*100
            l_vel = x*100
            r_dir = 1 * math.copysign(1, x)
            l_dir = 1 * math.copysign(1, x)
        elif x == 0 and y != 0:
            r_vel = y * 50
            r_dir = math.copysign(1, y)
            r_vel = y * 50
            r_dir = -1 *math.copysign(1, y)

        elif x != 0 and y != 0:
            if y < 0:
                incr_r = 0.8
                incr_l = 1
            else:
                incr_r = 1
                incr_l = 0.8
            r_vel = x * 100 * incr_r
            l_vel = x * 100 * incr_l
            r_dir = 1 * math.copysign(1, x)
            l_dir = 1 * math.copysign(1, x)
        velocity = [r_vel, l_vel]
        direction = [r_dir, l_dir]

        for i in range(2):
            self.dc_motor_array_msg.motors[i].id = i+1
            self.dc_motor_array_msg.motors[i].velocity = velocity[i]
            self.dc_motor_array_msg.motors[i].direction = direction[i]
        self.ros_pub_dc_motor_array.publish(self.dc_motor_array_msg)

    def run(self):
        """
        This function is used to run the node
        """
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rate.sleep()
        # clean up while shutdown
        rospy.loginfo("[INP] all done!")


if __name__ == '__main__':

    key_board_input = KeybardInput()
    try:
        key_board_input.run()
    except rospy.ROSInterruptException:
        pass
