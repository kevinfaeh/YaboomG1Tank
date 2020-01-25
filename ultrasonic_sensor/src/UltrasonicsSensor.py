# coding=utf-8
# import GPIO modules
import time
import RPi.GPIO as GPIO

class UltrasonicSensor:

    def __init__(self, ID, Trigger_PIN_OUT, Echo_PIN_IN):

        self.Trigger_PIN_OUT = Trigger_PIN_OUT
        self.Echo_PIN_IN = Echo_PIN_IN
        self.ID = ID
        GPIO.setup(Trigger_PIN_OUT, GPIO.OUT)
        GPIO.setup(Echo_PIN_IN, GPIO.IN)
        GPIO.output(Trigger_PIN_OUT, False)

    def measure(self):

        GPIO.output(self.Trigger_PIN_OUT, True)
        time.sleep(0.000015)
        GPIO.output(self.Trigger_PIN_OUT, False)

        # start clock
        start_time = time.time()
        while GPIO.input(self.Echo_PIN_IN) == 0:
            start_time = time.time()

        while GPIO.input(self.Echo_PIN_IN) == 1:
            end_time = time.time()

        # the difference to calculate the distance
        total_time = end_time - start_time
        # based on speed of light calcualte the distance in m
        distance = (total_time * 343) / 2

        # check if the values are reasonable
        if distance < 0.02 or (round(distance) > 3):
            return -1
        else:
            return round(distance, 3)
