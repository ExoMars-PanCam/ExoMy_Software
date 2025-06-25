#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import time

import Adafruit_PCA9685

class Ptu():
    """
    PTU class contains all functions to control the pan and tilt unit
    """

    # Define wheel names
    Pan, Tilt = range(0, 2)

    def __init__(self):
        # Set variables for the GPIO ptu pins
        self.pin_pan = rospy.get_param("pin_ptu_pan")
        self.pin_tilt = rospy.get_param("pin_ptu_tilt")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
        self.pwm.set_pwm_freq(50)  # Hz

        self.pan_pwm_neutral = rospy.get_param("ptu_pwm_neutral_pan")
        self.tilt_pwm_neutral = rospy.get_param("ptu_pwm_neutral_tilt")

        self.pan_pwm_range = rospy.get_param("ptu_pwm_range_pan")
        self.tilt_pwm_range = rospy.get_param("ptu_pwm_range_tilt")

        self.tilt_pwm_start = 185 #TODO! Move to config file

        self.wake() # First movement of the PTU

    def tilt_transition(self, start_pos, end_pos, upwards=True):
        """
        Slowly transition the tilt from start_pos to end_pos.
        """
        if upwards:
            increment = 1
        else:
            increment = -1

        for pos in range(start_pos, end_pos, increment):
            self.pwm.set_pwm(self.pin_tilt, 0, pos)

            time.sleep(0.03)

    def pan_transition(self, start_pos, end_pos, clockwise=True):
        """
        Slowly transition the pan from start_pos to end_pos.
        """
        if clockwise:
            increment = 2
        else:
            increment = -2

        for pos in range(start_pos, end_pos, increment):
            self.pwm.set_pwm(self.pin_pan, 0, pos)
            time.sleep(0.02)

    def wake(self):
        # Start with the tilt moving up and then down.
        self.tilt_transition(self.tilt_pwm_start, 425)
        time.sleep(0.5)
        # Transition back to the neutral position
        self.tilt_transition(445, self.tilt_pwm_neutral, upwards=False)
        time.sleep(1.0)

        # Transition from neutral clockwise to end position
        self.pan_transition(self.pan_pwm_neutral, 470, clockwise=True)
        time.sleep(0.5)
        
        #  Transition all the way anticlockwise to the end position
        self.pan_transition(455, 125, clockwise=False)
        time.sleep(0.5)
        self.pan_transition(125, self.pan_pwm_neutral, clockwise=True)
        time.sleep(2.0)
