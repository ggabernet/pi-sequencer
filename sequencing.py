#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
from picamera import PiCamera
import os
import random


class Sequencing:
    """Sequencing lego tiles."""

    def __init__(self, motor_in1=6, motor_in2=13, motor_in3=19, motor_in4=26, led_pin=17, button_pin=18, step_time=0.001):
        """
        Initializes sequencing class. Sequencer device comprises a camera, a white LED to provide light to the camera,
        a button to control the start of the sequencing process and a step motor (28BYJ-48, with driver ULN2003AN)
        to move the LEGO tiles in front of the camera.
        :param motor_in1: GPIO pin used as input for the stepper motor IN1.
        :param motor_in2: GPIO pin used as input for the stepper motor IN2.
        :param motor_in3: GPIO pin used as input for the stepper motor IN3.
        :param motor_in4: GPIO pin used as input for the stepper motor IN4.
        :param led_pin: GPIO pin used as output for lighting the LED.
        :param button_pin: GPIO pin used as input for the button.
        :param step_time: Step time to control the stepper motor speed.
        """
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)

        # Set GPIO pins
        self.in1_pin = motor_in1
        self.in2_pin = motor_in2
        self.in3_pin = motor_in3
        self.in4_pin = motor_in4
        self.led_pin = led_pin
        self.button_pin = button_pin


        # Set camera as PiCamera
        self.camera = PiCamera()

        # Set motor step time to control speed
        self.step_time = step_time

        # Set LedPin mode as output
        GPIO.setup(self.led_pin, GPIO.OUT)

        # Set BtnPin mode as input, and pull up to high level (3.3V)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Set LedPin high(3.3V) to off LED
        GPIO.output(self.led_pin, GPIO.HIGH)

        # Set motor pins as output
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        GPIO.setup(self.in3_pin, GPIO.OUT)
        GPIO.setup(self.in4_pin, GPIO.OUT)

        # Set motor to off at the beginning
        GPIO.output(self.in1_pin, False)
        GPIO.output(self.in2_pin, False)
        GPIO.output(self.in3_pin, False)
        GPIO.output(self.in4_pin, False)

        # Start camera preview
        self.camera.start_preview(fullscreen=False, window=(100, 20, 640, 480))

    def motor_step1(self):
        GPIO.output(self.in4_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in4_pin, False)

    def motor_step2(self):
        GPIO.output(self.in4_pin, True)
        GPIO.output(self.in3_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in4_pin, False)
        GPIO.output(self.in3_pin, False)

    def motor_step3(self):
        GPIO.output(self.in3_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in3_pin, False)

    def motor_step4(self):
        GPIO.output(self.in2_pin, True)
        GPIO.output(self.in3_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in2_pin, False)
        GPIO.output(self.in3_pin, False)

    def motor_step5(self):
        GPIO.output(self.in2_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in2_pin, False)

    def motor_step6(self):
        GPIO.output(self.in1_pin, True)
        GPIO.output(self.in2_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in1_pin, False)
        GPIO.output(self.in2_pin, False)

    def motor_step7(self):
        GPIO.output(self.in1_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in1_pin, False)

    def motor_step8(self):
        GPIO.output(self.in4_pin, True)
        GPIO.output(self.in1_pin, True)
        time.sleep(self.step_time)
        GPIO.output(self.in4_pin, False)
        GPIO.output(self.in1_pin, False)

    def motor_right(self, step):
        """
        Turns motor right a number of steps.
        :param step:
        :param step_time:
        :return:
        """
        for i in range(step):
            print("step right ",i)
            self.motor_step1()
            self.motor_step2()
            self.motor_step3()
            self.motor_step4()
            self.motor_step5()
            self.motor_step6()
            self.motor_step7()
            self.motor_step8()

    def motor_left(self, step):
        for i in range(step):
            print("step left ", i)
            self.motor_step8()
            self.motor_step7()
            self.motor_step6()
            self.motor_step5()
            self.motor_step4()
            self.motor_step3()
            self.motor_step2()
            self.motor_step1()

    def start_sequencing(self, ev=None):
        print("Starting: ")
        self.motor_right(3000)
        time.sleep(2)
        self.motor_left(200)
        time.sleep(2)
        print("Light on")
        GPIO.output(self.led_pin, 0)
        time.sleep(1)
        print("Starting sequencing")
        self.camera.start_recording('/home/pi/Desktop/video.h264')
        self.motor_left(2800)
        print("stopping sequencing")
        self.camera.stop_recording()
        print("light off")
        GPIO.output(self.led_pin, 1)

    def detect_button(self, time_sleep=1, bouncetime=200):
        """
        Detect pressing the button. Also wait for falling of the button and setting bouncetime to prevent the calling
        the function multiple times when the button is pressed.
        :param time_sleep: Time sleep for which to wait for bouncetime.
        :param bouncetime: Time set as bouncetime.
        :return:
        """
        GPIO.add_event_detect(self.button_pin, GPIO.FALLING, callback=self.start_sequencing, bouncetime=bouncetime)
        while True:
            time.sleep(time_sleep)   # Don't do anything

    def reset(self):
        """
        If program is stopped, GPIO outputs need to be set properly again. Turn led off.
        :return:
        """
        # Turn led off
        GPIO.output(self.led_pin, GPIO.HIGH)
        # Release GPIOs
        GPIO.cleanup()
        # Stop camera preview
        self.camera.stop_preview()

if __name__ == '__main__':     # Program start from here
    seq = Sequencing()
    try:
        seq.detect_button()
    except KeyboardInterrupt:  # When 'Ctrl+C' is pressed, the child program destroy() will be  executed.
        seq.reset()

