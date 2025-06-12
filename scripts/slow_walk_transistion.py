#!/usr/bin/env python3
import Adafruit_PCA9685
import time
from shutil import copyfile
import os
from config05_walk_motor_neutral import get_walking_motor_pins, get_walking_pwm_neutral_values

i2c_address=0x40 # For walking and PTU
i2c_busnum =1

# For most motors a pwm frequency of 50Hz is normal
pwm_frequency = 50.0  # Hz

# The cycle is the inverted frequency converted to milliseconds
cycle = 1.0/pwm_frequency * 1000.0  # ms

# The time the pwm signal is set to on during the duty cycle
on_time_1 = 2.4  # ms
on_time_2 = 1.5  # ms

# Duty cycle is the percentage of a cycle the signal is on
duty_cycle_1 = on_time_1/cycle
print('Duty cycle 1: {}'.format(duty_cycle_1))
duty_cycle_2 = on_time_2/cycle

# The PCA 9685 board requests a 12 bit number for the duty_cycle
value_1 = 275 #int(duty_cycle_1*4096.0)
value_2 = 325 #int(duty_cycle_2*4096.0)
neutral = 300  # The neutral position of the motor, which is the middle of the duty cycle

pin_name = 'pin_'
addr_name = 'addr_'
bus_name = 'bus_'

# Configure pwm method
pwm = Adafruit_PCA9685.PCA9685(address=i2c_address, busnum=i2c_busnum)
pwm.set_pwm_freq(pwm_frequency)

# First get the list of walking motor pins
walking_motor_pins = get_walking_motor_pins()
pwm_neutral_dict = get_walking_pwm_neutral_values()

print("Walking motor pins:", walking_motor_pins)
print("Walking motor neutral values:", pwm_neutral_dict)

def walk_transistion(start_pos, end_pos, stand):
    """
    Slowly transition the motors from start_pos to end_pos with the specified increment.
    """
    if stand:
        increment = -1
    else:
        increment = 1

    for pos in range(start_pos, end_pos, increment):
        for pin_name, pin_value in walking_motor_pins.items():
            pwm_neutral_name = pin_name.replace('pin_walk_', 'walk_pwm_neutral_')
            pwm_neutral_value = pwm_neutral_dict[pwm_neutral_name]
            dir = 1 if pin_name.endswith('l') else -1
            pwm_value = pwm_neutral_value + (dir * pos)

            pwm.set_pwm(pin_value, 0, pwm_value)

        time.sleep(0.05)  # Adjust the sleep time for smoother transitions

#walk_transistion(160, 0, True)  # Transition to stand
walk_transistion(0, 160, False)  # Transition to sit
#walk_transistion(-65, 165, False)  # Transition to wave position