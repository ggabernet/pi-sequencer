#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
from picamera import PiCamera
import os
import random

class Sequencing:
    """Defines steps for sequencing"""

    def __init__(self, motor_in1=6, motor_in2=13, motor_in3=19, motor_in4=26, led_pin=17, button_pin=18):
        """
        """
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        self.in1_pin = motor_in1
        self.in2_pin = motor_in2
        self.in3_pin = motor_in3
        self.in4_pin = motor_in4
        self.led_pin = led_pin
        self.button_pin = button_pin
        # Set LED status to 1
        self.led_status = 1
        # Set camera as PiCamera
        self.camera = PiCamera()
    
    def setup(self):
        # Set LedPin mode as output
        GPIO.setup(LedPin, GPIO.OUT)
        # Set BtnPin mode as input, and pull up to high level (3.3V)
        GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Set LedPin high(3.3V) to off LED
        GPIO.output(LedPin, GPIO.HIGH) 
        # Start camera preview
        camera.start_preview()

    def swLed(self, ev=None):
        global Led_status
        Led_status = not Led_status
        # switch led status(on-->off; off-->on)
        GPIO.output(LedPin, Led_status)
        if Led_status == 1:
            print ("starting recording")
            camera.stop_recording()
        else:
            print ("stopping recording")
            camera.start_recording('/home/pi/Desktop/video.h264')

    def loop(self):
        # wait for falling and set bouncetime to prevent calling the function multiple times when the button is pressed
        GPIO.add_event_detect(BtnPin, GPIO.FALLING, callback=swLed, bouncetime=200)
        while True:
            time.sleep(1)   # Don't do anything

    def destroy():
        GPIO.output(LedPin, GPIO.HIGH)     # led off
        GPIO.cleanup()                     # Release resource
        camera.stop_preview()

if __name__ == '__main__':     # Program start from here
    setup()
    try:
        loop()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        destroy()

