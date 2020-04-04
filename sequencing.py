#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
from picamera import PiCamera
import os
import random

#GPIO mode BCM
GPIO.setmode(GPIO.BCM)

# PINS for ULN2003A stepper motor driver
IN1 = 6
IN2 = 13
IN3 = 19
IN4 = 26

# PINS for LED
LedPin = 17

# PINS for botton
BtnPin = 18

# Set camera as PiCamera
camera = PiCamera()

def setup():
    # Set LedPin mode as output
    GPIO.setup(LedPin, GPIO.OUT)
    # Set BtnPin mode as input, and pull up to high level (3.3V)
    GPIO.setup(BtnPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # Set LED status to 1
    Led_status = 1
    # Set LedPin high(3.3V) to off LED
    GPIO.output(LedPin, GPIO.HIGH) 
    # Start camera preview
    camera.start_preview()

def swLed(ev=None):
    global Led_status
    Led_status = not Led_status
    GPIO.output(LedPin, Led_status)  # switch led status(on-->off; off-->on)
    if Led_status == 1:
        print ("starting recording")
        camera.stop_recording()
    else:
        print ("stopping recording")
        camera.start_recording('/home/pi/Desktop/video.h264')

def loop():
    GPIO.add_event_detect(BtnPin, GPIO.FALLING, callback=swLed, bouncetime=200) # wait for falling and set bouncetime to prevent the callback function from being called multiple times when the button is pressed
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

