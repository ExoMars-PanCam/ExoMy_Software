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
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)
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

    def transition(self, start_pos, end_pos, stand):
        """
        Slowly transition the motors from start_pos to end_pos with the specified increment.
        """
        if stand:
            increment = -1
        else:
            increment = 1

        for pos in range(start_pos, end_pos, increment):
            for wheel_name, motor_pin in self.pins['walk'].items():
                duty_cycle = self.walking_pwm_neutral[wheel_name] + self.wheel_directions[wheel_name] * pos
                self.pwm.set_pwm(motor_pin, 0, duty_cycle)

            time.sleep(0.08)

    def stand(self):
        # Raise up the robot to the lean position
        self.transition(160, 0, True)
        self.wave()

    def sit(self):
        # Sit the robot down
        self.transition(0, 160, False)

    def wave(self):
        # Transition from standing to -50 position to take weight off the fl wheel
        self.transition(0, -50, True)
        
        # Wave the fl wheel
        walk_fl = self.pins['walk'][self.FL]
        pwm_fl_neutral = self.walking_pwm_neutral[self.FL]

        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 50))
        time.sleep(1)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 170))
        time.sleep(2)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 130))
        time.sleep(0.3)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 170))
        time.sleep(0.3)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 130))
        time.sleep(0.3)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 170))
        time.sleep(2)
        self.pwm.set_pwm(walk_fl, 0, int(pwm_fl_neutral - 50))
        time.sleep(1)

        # Transition back to standing
        self.transition(-50, 0, False)