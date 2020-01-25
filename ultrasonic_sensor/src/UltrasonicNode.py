#!/usr/bin/python
# coding=utf-8
# Benötigte Module werden eingefügt und konfiguriert
import RPi.GPIO as GPIO
import rospy

from sensor_msgs.msg import Range
from UltrasonicsSensor import UltrasonicSensor


GPIO.setmode(GPIO.BCM)

ultrasonic_sensor = UltrasonicSensor(1, 1, 0)

rospy.init_node("ultrasonic")
rospy.loginfo("[Ultrasonic] starting node")
rate = rospy.Rate(10)

data_publisher = rospy.Publisher("ultrasonic/sensordata", Range, queue_size=3)
message = Range()
message.field_of_view = (10.0/180.0) * 3.141459265
message.min_range = 0.02
message.max_range = 2
message.header.frame_id = "/sonar_ranger"
# main
try:
    while not rospy.is_shutdown():
        result = ultrasonic_sensor.measure()
        message.range = result
        data_publisher.publish(message)
        rate.sleep()
    GPIO.cleanup()
    rospy.loginfo("[Ultrasonic] all done!")


except rospy.ROSInterruptException:
    GPIO.cleanup()
