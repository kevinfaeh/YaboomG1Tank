#!/usr/bin/python
# coding=utf-8
"""
This class is used to create as DC motor object and control it.
"""

import RPi.GPIO as GPIO


class DC_Motor:
    """
    This class holds all functionality for the DC motors.
    """

    def __init__(self, id, PIN_IN1, PIN_IN2, PIN_ENX, pwm_frequency):
        """
        This function initializes a DC Motor object.
        :param id: The id of the motor.
        :type id: int
        :param PIN_IN1: Input 1 for motor controller of BST-4WD (e.g. 20 or 19)
        :type PIN_IN1: int
        :param PIN_IN2: Input 2 for motor controller of BST-4WD (e.g. 21 or 26)
        :type PIN_IN2: int
        :param PIN_ENX: Digital IO for motor controller of BST-4WD (e.g. 16 or 13)
        :type PIN_ENX: int
        :param pwm_frequency: The frequency for pulse width modulation (2000 Hz)
        :type pwm_frequency: int
        :return:
        """
        self.id = id

        # Set the GPIO port to BCM encoding mode.
        GPIO.setmode(GPIO.BCM)
        # Ignore warning information
        GPIO.setwarnings(False)

        self.IN1 = PIN_IN1
        self.IN2 = PIN_IN2
        self.ENX = PIN_ENX

        self.pins = [self.IN1, self.IN2, self.ENX]

        self.pwm_frequency = pwm_frequency

        GPIO.setup(self.ENX, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.IN2, GPIO.OUT, initial=GPIO.LOW)

        # Set the PWM pin and frequency is 2000hz
        self.pwm_ENX = GPIO.PWM(self.ENX, self.pwm_frequency)
        self.pwm_ENX.start(0)

        self.velocity = 0
        self.direction = 0

    def run_motor(self, velocity, direction):
        """
        This function moves the motor with the desired velocity in the desired direction
        :param velocity: the velocity command [0, 100]
        :type velocity: float
        :param direction: the direction of the motor [-1, 1]
        :type direction: int
        """
        if velocity == 0:
            self.stop_motor()
        else:
            if direction == -1:
                GPIO.output(self.IN1, GPIO.LOW)
                GPIO.output(self.IN2, GPIO.HIGH)
                self.pwm_ENX.ChangeDutyCylce(velocity)
                self.direction = direction
                self.velocity = velocity
            else:
                GPIO.output(self.IN1, GPIO.HIGH)
                GPIO.output(self.IN2, GPIO.LOW)
                self.pwm_ENX.ChangeDutyCylce(velocity)
                self.direction = direction
                self.velocity = velocity

    def stop_motor(self):
        """
        This function stops the motor.
        """
        GPIO.output(self.IN1, GPIO.LOW)
        GPIO.output(self.IN2, GPIO.LOW)
        self.pwm_ENX.ChangeDutyCylce(0)
        self.velocity = 0
        self.direction = 0

    def get_information(self):
        """
        This function returns all information about the motor.
        :return: list of data: id, pwm_frequency, pins, direction, velocity
        :rtype: list
        """

        return [self. id, self.pwm_frequency, self.pins, self.direction, self.velocity]

    def get_velocity_direction(self):
        """
        This function returns the actual velocity and direction
        :return: list of direction and velocity
        :rtype list
        """

        return [self.direction, self.velocity]

