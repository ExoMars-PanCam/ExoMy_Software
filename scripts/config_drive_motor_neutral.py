#!/usr/bin/env python3
import Adafruit_PCA9685
import yaml
import time
import os
from config_drive_pins import Motor, update_config_file

config_filename = '../config/exomy.yaml'


def get_drive_pins():
    pin_list = []
    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file,Loader=yaml.FullLoader)

    for key, value in param_dict.items():
        if('pin_drive' in str(key)):
            pin_list.append(value)
    return pin_list

def get_drive_pwm_neutral():

    with open(config_filename, 'r') as file:
        param_dict = yaml.load(file,Loader=yaml.FullLoader)

    for key, value in param_dict.items():
        if('drive_pwm_neutral' in str(key)):
            return value                    

    default_value = 300
    print('The parameter drive_pwm_neutral could not be found in the exomy.yaml \n')
    print('It was set to the default value: '+ default_value + '\n')
    return default_value

if __name__ == "__main__":
    print(
        '''
███████╗██╗  ██╗ ██████╗ ███╗   ███╗██╗   ██╗    ██████╗     ██████╗ 
██╔════╝╚██╗██╔╝██╔═══██╗████╗ ████║╚██╗ ██╔╝    ╚════██╗   ██╔═████╗
█████╗   ╚███╔╝ ██║   ██║██╔████╔██║ ╚████╔╝      █████╔╝   ██║██╔██║
██╔══╝   ██╔██╗ ██║   ██║██║╚██╔╝██║  ╚██╔╝      ██╔═══╝    ████╔╝██║
███████╗██╔╝ ██╗╚██████╔╝██║ ╚═╝ ██║   ██║       ███████╗██╗╚██████╔╝
╚══════╝╚═╝  ╚═╝ ╚═════╝ ╚═╝     ╚═╝   ╚═╝       ╚══════╝╚═╝ ╚═════╝ 
                                                                     
        '''
    )
    print(
        '''
This script helps you to set the neutral values of PWM of the driving motors correctly.
It will send the intended signal for "not moving" to all the motors.
On each motor you have to turn the correction screw until the motor really stands still.
        '''
    )

    if not os.path.exists(config_filename):
        print("exomy.yaml does not exist. Finish config_drive_pins.py to generate it.")
        exit()

    #Default Motor Hat Address for Drive motors
    pwm = Adafruit_PCA9685.PCA9685(address=0x40, busnum=1)

    '''
    The drive_pwm_neutral value is determined from the exomy.yaml file.
    But it can be also calculated from the values of the PWM board and motors, 
    like shown in the following calculation:

    # For most motors a pwm frequency of 50Hz is normal
    pwm_frequency = 50.0  # Hz
    pwm.set_pwm_freq(pwm_frequency)

    # The cycle is the inverted frequency converted to milliseconds
    cycle = 1.0/pwm_frequency * 1000.0  # 20 ms

    # The time the pwm signal is set to on during the duty cycle
    on_time = 1.5  # ms

    # Duty cycle is the percentage of a cycle the signal is on
    duty_cycle = on_time/cycle # 0.075

    # The PCA 9685 board requests a 12 bit number for the duty_cycle
    value = int(duty_cycle*4096.0) # 307
    '''

    value = get_drive_pwm_neutral()
    pin_list = get_drive_pins()

    for pin in pin_list:
        pwm.set_pwm(pin, 0, value)
        time.sleep(0.1)

    input('Press any button if you are done to complete configuration\n')

    for pin in pin_list:
        pwm.set_pwm(pin, 0, 0)
