#!/usr/bin/python
# coding=utf-8
# Benötigte Module werden eingefügt und konfiguriert
import RPi.GPIO as GPIO
import rospy

from sensor_msgs.msg import Range
from UltrasonicsSensor import UltrasonicSensor


GPIO.setmode(GPIO.BCM)

ultrasonic_sensor = UltrasonicSensor(1, 1, 0)

rospy.loginfo("[Ultrasonic] starting node")
rospy.init_node("ultrasonic")
rate = rospy.Rate(20)

data_publisher = rospy.Publisher("ultrasonic/sensordata", Range, queue_size=3)
message = Range()
message.min_range = 0.02
message.max_range = 2
message.header.frame_id = "/sonar_ranger"
# main
try:
    while not rospy.is_shutdown():
        result = ultrasonic_sensor.measure()
        message.range = result
        message.field_of_view = 0
        data_publisher.publish(message)
        rate.sleep()
    GPIO.cleanup()
    rospy.loginfo("[Ultrasonic] all done!")


except rospy.ROSInterruptException:
    GPIO.cleanup()
