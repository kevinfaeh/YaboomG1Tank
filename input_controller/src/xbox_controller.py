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

        self.servo_ids = []
        self.servo_angles = []

        self.ros_sub_servo_array = rospy.Subscriber("/low_level_ctrl/servo_array", ServoMotorArray, self.store_servo_state)

        self.servo_array_msg = ServoMotorArray()
        self.servo_array_msg.servos = []
        for i in range(3):
            self.servo_array_msg.servos.append(ServoMotor_msg())

        self.dc_motor_array_msg = DC_MotorArray()
        self.dc_motor_array_msg.motors = []
        for i in range(2):
            self.dc_motor_array_msg.motors.append(DC_Motor_msg())

        rospy.loginfo("[XBOX] node initialization finished")

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

    def publish_servo_message(self, servo_id, value):
        """
        This function publishes the servo message with the update for the index and angle
        :param servo_id: the id of the servo
        :param value: the value to change the current angle
        """
        for i in range(3):
            if i == servo_id - 1:
                self.servo_array_msg.servos[i].id = self.servo_ids[i]
                self.servo_array_msg.servos[i].angle = self.servo_angles[i] + value
            else:
                self.servo_array_msg.servos[i].id = self.servo_ids[i]
                self.servo_array_msg.servos[i].angle = self.servo_angles[i]

        self.ros_pub_servo_array.publish(self.servo_array_msg)

    def publish_dc_motor_message(self, velocity, direction):
        """
        This function publishes the dc motor message
        :param velocity: The list with the velocities for the motors
        :type velocity: list
        :param direction: the list with the directions for the motors
        :type direction: list
        """
        for i in range(2):
            self.dc_motor_array_msg.motors[i].id = i+1
            self.dc_motor_array_msg.motors[i].velocity = velocity[i]
            self.dc_motor_array_msg.motors[i].direction = direction[i]
        self.ros_pub_dc_motor_array.publish(self.dc_motor_array_msg)

    def set_servo_from_controller(self, event_state, event_state_abs, servo_id, direction):
        """
        This function sets the servo message from the controller command
        :param event_state: the event state
        :type event_state: the value from the controller
        :param event_state_abs: the absolute value fo the event state
        :type event_state_abs: the absolute value of the controller
        :param servo_id: the id of the servo
        :type servo_id: int
        :param direction: the direction to move the motor
        :type direction: int
        """
        sign = math.copysign(1, event_state)

        if event_state_abs >= 5000:
            if event_state_abs > 25000:
                self.publish_servo_message(servo_id, 6 * sign * direction)
            elif event_state_abs > 12000:
                self.publish_servo_message(servo_id, 3 * sign * direction)
            else:
                self.publish_servo_message(servo_id, 1 * sign * direction)

    def set_servo_from_tab(self, event_state_abs, servo_id, direction):
        """
        This function sets the servo message from the controller command
        :param event_state: the event state
        :type event_state: the value from the controller
        :param event_state_abs: the absolute value fo the event state
        :type event_state_abs: the absolute value of the controller
        :param servo_id: the id of the servo
        :param direction: the direction to move the servo
        :type direction: int
        :type servo_id: int
        """
        if event_state_abs >= 100:
            if event_state_abs > 800:
                self.publish_servo_message(servo_id, 6 * direction)
            elif event_state_abs > 500:
                self.publish_servo_message(servo_id, 3 * direction)
            else:
                self.publish_servo_message(servo_id, 1 * direction)

    @staticmethod
    def calculate_velocity_and_direction(x, y):
        """
        This function calculates the velocity and direction of the two dc motors from the x and y state of the joy stick
        :param x: the x value
        :type x: float
        :param y: the y value
        :type y: float
        :return: list with velocities and directions
        :rtype: list, list
        """
        if abs(x) >= abs(y):
            r_dir = math.copysign(1, x)
            l_dir = math.copysign(1, x)
            if y >= 0:
                r_vel = abs(x)
                l_vel = abs(x) - y
            else:
                r_vel = abs(x) - abs(y)
                l_vel = abs(x)

        else:
            if y >= 0:
                r_dir = math.copysign(1, x)
                l_dir = -1 *math.copysign(1, x)
                r_vel = y
                l_vel = y - abs(x)
            else:
                r_dir = -1 * math.copysign(1, x)
                l_dir = math.copysign(1, x)
                r_vel = abs(y) - abs(x)
                l_vel = abs(y)

        velocity = [r_vel, l_vel]
        direction = [r_dir, l_dir]

        return velocity, direction

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
            event_state_abs = abs(event_state)

        x = x_last
        y = y_last
        if event_code == "ABS_Y":
            if event_state_abs > 5000:
                x = -1 *event_state / 33000.0 * 100
            else:
                x = 0.0
            rospy.loginfo(x)
        elif event_code == "ABS_X":
            if event_state_abs > 5000:
                y = event_state / 33000.0 * 100
            else:
                y = 0.0
            rospy.loginfo(y)
        elif event_code == "ABS_RX":
            self.set_servo_from_controller(event_state, event_state_abs, 2, -1)
        elif event_code == "ABS_RY":
            self.set_servo_from_controller(event_state, event_state_abs, 3, 1)
        elif event_code == "ABS_RZ":
            self.set_servo_from_tab(event_state_abs, 1, -1)
        elif event_code == "ABS_Z":
            self.set_servo_from_tab(event_state_abs, 1, 1)

        velocity, direction = self.calculate_velocity_and_direction(x, y)

        self.publish_dc_motor_message(velocity, direction)

        return x, y

    def run(self):
        """
        This function is used to run the node
        """
        rate = rospy.Rate(1000)
        x_last = 0
        y_last = 0
        while not rospy.is_shutdown():
            x_last, y_last = self.set_motors_from_xbox(x_last, y_last)
            rate.sleep()
        # clean up while shutdown
        rospy.loginfo("[XBOX] all done!")


if __name__ == '__main__':

    game_pad_input = GamePadInput()
    try:
        game_pad_input.run()
    except rospy.ROSInterruptException:
        pass
