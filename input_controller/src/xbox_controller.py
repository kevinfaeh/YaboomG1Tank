#!/usr/bin/env python
"""
This file is used to steer the robot with an xbox one controller.
"""
import rospy
import math
import time


from inputs import get_gamepad
from geometry_msgs.msg import Twist
from input_controller.msg import DC_Motor_msg
from input_controller.msg import DC_MotorArray
from input_controller.msg import ServoMotor_msg
from input_controller.msg import ServoMotorArray


class GamePadInput:
    """
    This class is used to control the robot with game pad Input.
    """

    def __init__(self):
        """
        This function initializes gamepad controller.
        """
        self.node_name = "xbox_ctrl"
        rospy.init_node(self.node_name)

        rospy.loginfo("[XBOX] initializing controller")

        self.ros_pub_servo_array = rospy.Publisher("/cmd_servo_array", ServoMotorArray, queue_size=1)
        self.ros_pub_dc_motor_array = rospy.Publisher("/cmd_dc_motor_array", DC_MotorArray, queue_size=1)
        rospy.loginfo("[XBOX] initalized publisher")

        self.servo_array_msg = ServoMotorArray()
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.dc_motor_array_msg = DC_MotorArray()
        self.dc_motor_array_msg.motors = []
        for i in range(2):
            self.dc_motor_array_msg.motors.append(DC_Motor_msg())

        rospy.loginfo("[XBOX] node initialization finished")

    def set_motors_from_xbox(self, x_last, y_last):
        """
        This function converst the twist message to a motor command
        :param x_last: the last input for x state
        :type x_last: float
        :param y_last: the last input for y state
        :type y_last: float
        :return: the actual x and y state
        :rtype: float, float
        """

        events = get_gamepad()
        for event in events:
            event_code = event.code
            event_state = float(event.state)

        x = x_last
        y = y_last
        if event_code == "ABS_Y":
            if abs(event_state) > 5000:
                x = -1 *event_state / 33000.0 * 100
            else:
                x = 0.0
            rospy.loginfo(x)
        elif event_code == "ABS_RY":
            if abs(event_state) > 5000:
                y = -1 *event_state / 33000.0 * 100
            else:
                y = 0.0
            rospy.loginfo(y)

        r_vel = abs(x)
        r_dir = math.copysign(1, x)
        l_vel = abs(y)
        l_dir = math.copysign(1, y)

        velocity = [r_vel, l_vel]
        direction = [r_dir, l_dir]

        for i in range(2):
            self.dc_motor_array_msg.motors[i].id = i+1
            self.dc_motor_array_msg.motors[i].velocity = velocity[i]
            self.dc_motor_array_msg.motors[i].direction = direction[i]
        self.ros_pub_dc_motor_array.publish(self.dc_motor_array_msg)

        return x, y

    def set_motors_from_xbox_2(self, x_last, y_last):
        """
        This function converst the twist message to a motor command
        :param x_last: the last input for x state
        :type x_last: float
        :param y_last: the last input for y state
        :type y_last: float
        :return: the actual x and y state
        :rtype: float, float
        """

        events = get_gamepad()
        for event in events:
            event_code = event.code
            event_state = float(event.state)

        x = x_last
        y = y_last
        if event_code == "ABS_Y":
            if abs(event_state) > 5000:
                x = -1 *event_state / 33000.0 * 100
            else:
                x = 0.0
            rospy.loginfo(x)
        elif event_code == "ABS_X":
            if abs(event_state) > 5000:
                y = event_state / 33000.0 * 100
            else:
                y = 0.0
            rospy.loginfo(y)

        if abs(x) >= abs(y):
            r_dir = math.copysign(1, x)
            l_dir = math.copysign(1, x)
            if y >= 0:
                r_vel = abs(x)
                l_vel = abs(x) - y
            elif y <= 0:
                r_vel = abs(x) - abs(y)
                l_vel = abs(x)

        else:
            if y >= 0:
                r_dir = math.copysign(1, x)
                l_dir = -1 *math.copysign(1, x)
                r_vel = y
                l_vel = y - abs(x)
            elif y <= 0:
                r_dir = -1 * math.copysign(1, x)
                l_dir = math.copysign(1, x)
                r_vel = abs(y) - abs(x)
                l_vel = abs(y)

        velocity = [r_vel, l_vel]
        direction = [r_dir, l_dir]

        for i in range(2):
            self.dc_motor_array_msg.motors[i].id = i+1
            self.dc_motor_array_msg.motors[i].velocity = velocity[i]
            self.dc_motor_array_msg.motors[i].direction = direction[i]
        self.ros_pub_dc_motor_array.publish(self.dc_motor_array_msg)

        return x, y

    def run(self):
        """
        This function is used to run the node
        """
        rate = rospy.Rate(1000)
        x_last = 0
        y_last = 0
        while not rospy.is_shutdown():
            x_last, y_last = self.set_motors_from_xbox_2(x_last, y_last)
            rate.sleep()
        # clean up while shutdown
        rospy.loginfo("[INP] all done!")


if __name__ == '__main__':

    game_pad_input = GamePadInput()
    try:
        game_pad_input.run()
    except rospy.ROSInterruptException:
        pass
