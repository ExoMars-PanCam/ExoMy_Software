#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

import time

import Adafruit_PCA9685

class Walking():
    """
    Walking class contains all functions to control the walking of legs
    """

    # Define wheel names
    FL, FR, CL, CR, RL, RR = range(0, 6)

    # Defines the simple rotation direction for the Rover to stand-up
    wheel_directions = [1, -1, 1, -1, 1, -1]

    # 1 fl-||-fr 2
    #      ||
    # 3 cl-||-cr 4
    # 5 rl====rr 6

    def __init__(self):
        # Dictionary containing the walking pins
        self.pins = {
            'walk': {}
        }

        # Set variables for the GPIO walking pins
        self.pins['walk'][self.FL] = rospy.get_param("pin_walk_fl")
        self.pins['walk'][self.FR] = rospy.get_param("pin_walk_fr")
        self.pins['walk'][self.CL] = rospy.get_param("pin_walk_cl")
        self.pins['walk'][self.CR] = rospy.get_param("pin_walk_cr")
        self.pins['walk'][self.RL] = rospy.get_param("pin_walk_rl")
        self.pins['walk'][self.RR] = rospy.get_param("pin_walk_rr")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
        self.pwm.set_pwm_freq(50)  # Hz

        self.walking_pwm_neutral = [None] * 6
        self.walking_pwm_range = [None] * 6

        self.walking_pwm_neutral[self.FL] = rospy.get_param("walk_pwm_neutral_fl")
        self.walking_pwm_neutral[self.FR] = rospy.get_param("walk_pwm_neutral_fr")
        self.walking_pwm_neutral[self.CL] = rospy.get_param("walk_pwm_neutral_cl")
        self.walking_pwm_neutral[self.CR] = rospy.get_param("walk_pwm_neutral_cr")
        self.walking_pwm_neutral[self.RL] = rospy.get_param("walk_pwm_neutral_rl")
        self.walking_pwm_neutral[self.RR] = rospy.get_param("walk_pwm_neutral_rr")

        self.walking_pwm_range[self.FL] = rospy.get_param("walk_pwm_range_fl")
        self.walking_pwm_range[self.FR] = rospy.get_param("walk_pwm_range_fr")
        self.walking_pwm_range[self.CL] = rospy.get_param("walk_pwm_range_cl")
        self.walking_pwm_range[self.CR] = rospy.get_param("walk_pwm_range_cr")
        self.walking_pwm_range[self.RL] = rospy.get_param("walk_pwm_range_rl")
        self.walking_pwm_range[self.RR] = rospy.get_param("walk_pwm_range_rr")

        self.stand()

        self.sit()

    def stand(self):
        # Raise up the robot
  
        for percent in range(0, 100):
            time.sleep(0.1)
            for wheel_name, motor_pin in self.pins['walk'].items():
                duty_cycle = int(self.walking_pwm_neutral[wheel_name] * percent)
                self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def sit(self):
        # Sit the robot down

        for percent in range(0, 100):
            time.sleep(0.1)
            for wheel_name, motor_pin in self.pins['walk'].items():
                duty_cycle = int(self.walking_pwm_neutral[wheel_name] + self.walking_pwm_range[wheel_name] * self.wheel_directions[wheel_name] * percent)
                self.pwm.set_pwm(motor_pin, 0, duty_cycle)