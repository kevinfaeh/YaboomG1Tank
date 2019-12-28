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

    def __init__(self, id,  servo_pin, pwm_frequency, init_angle):
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

        # Set the GPIO port to BCM encoding mode.
        #GPIO.setmode(GPIO.BCM)
        # Ignore warning information
        #GPIO.setwarnings(False)

        self.pi = pigpio.pi()
        self.pi.set_mode(servo_pin, pigpio.OUTPUT)

        self.servo_pin = servo_pin
        self.pwm_frequency = pwm_frequency

        #GPIO.setup(self.servo_pin, GPIO.OUT)

        #self.pwm_servo = GPIO.PWM(servo_pin, pwm_frequency)
        #self.pwm_servo.start(0)
        self.angle = init_angle
        self.set_angle(init_angle)

    def set_angle(self, angle):
        """
        This function sets the angle of the servo motor
        :param angle: the angle to set the servo
        :type angle: float
        """
        #self.pwm_servo = GPIO.PWM(self.servo_pin, self.pwm_frequency)
        #self.pwm_servo.ChangeDutyCycle(2.5 + 10 * angle / 180)
        self.pi.set_servo_pulsewidth(self.servo_pin, 500 + 2000 * angle / 180)
        self.angle = angle

    def stop_motor(self):
        """
        This function stops the motor
        """
        #self.pwm_servo.stop()
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

