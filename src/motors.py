#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import math

import time

import Adafruit_PCA9685


class Motors():
    '''
    Motors class contains all functions to control the steering and driving
    '''

    # Define wheel names
    FL, FR, CL, CR, RL, RR = range(0, 6)

    # Motor commands are assuming positiv=driving_forward, negative=driving_backwards.
    # The driving direction of the left side has to be inverted for this to apply to all wheels.
    wheel_directions = [-1, 1, -1, 1, -1, 1]

    # 1 fl-||-fr 2
    #      ||
    # 3 cl-||-cr 4
    # 5 rl====rr 6

    def __init__(self):

        # Dictionary containing the pins of all drive motors
        self.pins = {
            'drive': {},
            'steer': {}
        }

        # Set variables for the GPIO motor pins
        self.pins['drive'][self.FL] = rospy.get_param("pin_drive_fl")
        self.pins['steer'][self.FL] = rospy.get_param("pin_steer_fl")

        self.pins['drive'][self.FR] = rospy.get_param("pin_drive_fr")
        self.pins['steer'][self.FR] = rospy.get_param("pin_steer_fr")

        self.pins['drive'][self.CL] = rospy.get_param("pin_drive_cl")
        self.pins['steer'][self.CL] = rospy.get_param("pin_steer_cl")

        self.pins['drive'][self.CR] = rospy.get_param("pin_drive_cr")
        self.pins['steer'][self.CR] = rospy.get_param("pin_steer_cr")

        self.pins['drive'][self.RL] = rospy.get_param("pin_drive_rl")
        self.pins['steer'][self.RL] = rospy.get_param("pin_steer_rl")

        self.pins['drive'][self.RR] = rospy.get_param("pin_drive_rr")
        self.pins['steer'][self.RR] = rospy.get_param("pin_steer_rr")

        # PWM characteristics
        self.pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=1)
        self.pwm.set_pwm_freq(50)  # Hz

        self.steering_pwm_neutral = [None] * 6
        self.steering_pwm_range = [None] * 6
        
        self.steering_pwm_neutral[self.FL] = rospy.get_param("steer_pwm_neutral_fl")
        self.steering_pwm_neutral[self.FR] = rospy.get_param("steer_pwm_neutral_fr")
        self.steering_pwm_neutral[self.CL] = rospy.get_param("steer_pwm_neutral_cl")
        self.steering_pwm_neutral[self.CR] = rospy.get_param("steer_pwm_neutral_cr")
        self.steering_pwm_neutral[self.RL] = rospy.get_param("steer_pwm_neutral_rl")
        self.steering_pwm_neutral[self.RR] = rospy.get_param("steer_pwm_neutral_rr")

        self.steering_pwm_range[self.FL] = rospy.get_param("steer_pwm_range_fl")
        self.steering_pwm_range[self.FR] = rospy.get_param("steer_pwm_range_fr")
        self.steering_pwm_range[self.CL] = rospy.get_param("steer_pwm_range_cl")
        self.steering_pwm_range[self.CR] = rospy.get_param("steer_pwm_range_cr")
        self.steering_pwm_range[self.RL] = rospy.get_param("steer_pwm_range_rl")
        self.steering_pwm_range[self.RR] = rospy.get_param("steer_pwm_range_rr")

        self.driving_pwm_neutral = rospy.get_param("drive_pwm_neutral")

        self.driving_pwm_range = rospy.get_param("drive_pwm_range")

        # Set steering motors to neutral values (straight)
        for wheel_name, motor_pin in self.pins['steer'].items():
            self.pwm.set_pwm(motor_pin, 0,
                             self.steering_pwm_neutral[wheel_name])
            time.sleep(0.1)

        ## For point turns
        # x = axes distance
        # y = axes width
        
        # Rear (r = rear)
        self.wheel_rx = 16.0
        self.wheel_ry = 27.0
        
        # Front (f = front)
        self.wheel_fx = 16.0
        self.wheel_fy = 27.0
        self.point_turn_angle = int(math.degrees(math.atan((self.wheel_rx+self.wheel_fx) / self.wheel_ry)))
        self.point_turn_angle_center = int(math.degrees(math.atan((((self.wheel_rx+self.wheel_fx) / 2 ) - self.wheel_fx) / (self.wheel_ry / 2))))

        # self.wiggle()

    def wiggle(self):
        time.sleep(0.2)
        
        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] + self.steering_pwm_range[self.FL] * 0.3))
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] + self.steering_pwm_range[self.FR] * 0.3))
        time.sleep(0.5)

        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL] - self.steering_pwm_range[self.FL] * 0.3))
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR] - self.steering_pwm_range[self.FR] * 0.3))
        time.sleep(0.5)

        self.pwm.set_pwm(self.pins['steer'][self.FL], 0,
                         int(self.steering_pwm_neutral[self.FL]))
        self.pwm.set_pwm(self.pins['steer'][self.FR], 0,
                         int(self.steering_pwm_neutral[self.FR]))
        time.sleep(0.2)

    def setSteering(self, steering_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['steer'].items():
            duty_cycle = int(
                self.steering_pwm_neutral[wheel_name] + steering_command[wheel_name]/90.0 * self.steering_pwm_range[wheel_name])

            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def setDriving(self, driving_command):
        # Loop through pin dictionary. The items key is the wheel_name and the value the pin.
        for wheel_name, motor_pin in self.pins['drive'].items():
            duty_cycle = int(self.driving_pwm_neutral +
                             driving_command[wheel_name]/100.0 * self.driving_pwm_range * self.wheel_directions[wheel_name])

            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def stopMotors(self):
        # Set driving wheels to neutral position to stop them
        duty_cycle = int(self.driving_pwm_neutral)

        for wheel_name, motor_pin in self.pins['drive'].items():
            self.pwm.set_pwm(motor_pin, 0, duty_cycle)

    def crabbing_drive(self, angle, duration=0.0):

        angle_array = [angle]*6
        self.setSteering(angle_array)
        time.sleep(1.0)
        drive_speed = 25
        driving_array = [drive_speed]*6
        self.setDriving(driving_array)
        time.sleep(duration)
        
        # Stop driving
        self.setDriving([0, 0, 0, 0, 0, 0])

        # Reset crabbing
        self.setSteering([0, 0, 0, 0, 0, 0])
        time.sleep(1.0)

    def point_turn(self, duration=5, clockwise=True):
        """
        Perform a point turn for the specified duration.
        If clockwise is True, the robot turns clockwise, otherwise counter-clockwise.
        """
        steering_angles = [0]*6
        motor_speeds = [0]*6
        
        #For ExoMy approx. 55 degree
        steering_angles[self.FL] = self.point_turn_angle
        steering_angles[self.FR] = -self.point_turn_angle
        steering_angles[self.CL] = self.point_turn_angle_center
        steering_angles[self.CR] = -self.point_turn_angle_center
        steering_angles[self.RL] = -self.point_turn_angle
        steering_angles[self.RR] = self.point_turn_angle

        self.setSteering(steering_angles)

        # Delay before starting to drive
        time.sleep(1.0)

        # Set rotation speed
        v = 50  
        outer_turning_radius = math.sqrt(math.pow(self.wheel_rx+self.wheel_fx,2) + math.pow(self.wheel_ry,2)) / 2
        inner_turning_radius = math.sqrt(math.pow(((self.wheel_rx+self.wheel_fx) / 2 ) - self.wheel_rx,2) + math.pow((self.wheel_ry / 2),2))
        
        v_outer = v
        v_inner = int(v*inner_turning_radius/outer_turning_radius)
            
        if clockwise:
            # Right turn
            motor_speeds[self.FL] = v_outer
            motor_speeds[self.FR] = -v_outer
            motor_speeds[self.CL] = v_inner
            motor_speeds[self.CR] = -v_inner
            motor_speeds[self.RL] = v_outer
            motor_speeds[self.RR] = -v_outer
        else:
            # Left turn
            motor_speeds[self.FL] = -v_outer*1.125
            motor_speeds[self.FR] = v_outer*1.125
            motor_speeds[self.CL] = -v_inner*1.125
            motor_speeds[self.CR] = v_inner*1.125
            motor_speeds[self.RL] = -v_outer*1.125
            motor_speeds[self.RR] = v_outer*1.125

        self.setDriving(motor_speeds)
        time.sleep(duration)

        # Stop driving
        self.setDriving([0, 0, 0, 0, 0, 0])
        # Return angles to neutral
        self.setSteering([0, 0, 0, 0, 0, 0])

    def straight_drive(self, duration=5.0, forward=True):
        """
        Drive straight for the specified duration.
        If forward is True, the robot drives forward, otherwise backward.
        """
        driving_array = [100]*6 if forward else [-100]*6
        self.setDriving(driving_array)
        time.sleep(duration)
        
        # Stop driving
        self.setDriving([0, 0, 0, 0, 0, 0])