#!/usr/bin/python
# coding=utf-8
"""
This class is used to create as Servo motor object and control it.
"""

#import RPi.GPIO as GPIO
import pigpio


class ServoMotor:
    """
    This class describes all functionality for a servo motor.
    """

    def __init__(self, id,  servo_pin, pwm_frequency, init_angle, angle_min, angle_max):
        """
        This function initializes an Servo Motor object.
        :param id: The id of the motor.
        :type id: int
        :param servo_pin: the pin number of the servo (e.g 23)
        :type servo_pin: int
        :param pwm_frequency: pulse width modulation (e.g. 50 Hz).
        :type pwm_frequency: int
        """
        self.id = id

        self.pi = pigpio.pi()
        self.pi.set_mode(servo_pin, pigpio.OUTPUT)

        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency

        self.angle_min = angle_min
        self.angle_max = angle_max

        self.angle = init_angle
        self.set_angle(init_angle)

    def set_angle(self, angle):
        """
        This function sets the angle of the servo motor
        :param angle: the angle to set the servo
        :type angle: float
        """
        if angle >= self.angle_max:
            angle = self.angle_max
        elif angle <= self.angle_min:
            angle = self.angle_min

        self.pi.set_servo_pulsewidth(self.servo_pin, 500 + 2000 * angle / 180)
        self.angle = angle

    def stop_motor(self):
        """
        This function stops the motor
        """
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        self.pi.stop()

    def get_angle(self):
        """
        This function returns the actual angle.
        :return: the actual angle
        :rtype float
        """
        return self.angle

    def get_information(self):
        """
        This function returns all information about the motor.
        :return: list of data: id, pwm_frequency, pin number, angle
        :rtype: list
        """

        return [self.id, self.pwm_frequency, self.servo_pin, self.angle]

