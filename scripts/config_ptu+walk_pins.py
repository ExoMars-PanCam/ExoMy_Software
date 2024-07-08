#!/usr/bin/env python3
import Adafruit_PCA9685
import time
from shutil import copyfile
import os
from config_drive_pins import Motor, update_config_file

# BJW: This file to be run after the config_drive_pins.py
WALK_MOTOR, PTU_MOTOR = [2, 3]

pos_names = {
    1: 'fl',
    2: 'fr',
    3: 'cl',
    4: 'cr',
    5: 'rl',
    6: 'rr',
    7: 'pan',
    8: 'tilt',
}

pin_dict = {

}

# Found using the pi command 'sudo i2cdetect -y 1' with the 1 being bus
# number
address=0x42
busnum =1

def print_exomy_layout():
    print(
        '''
        1 fl-||-fr 2
             ||
        3 cl-||-cr 4
        5 rl====rr 6
        
        7 <--->
        
        8 ^
          |
          v
        '''
    )


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
###############
Motor Configuration

This scripts leads you through the configuration of the motors used on the 
new motor controller hat.
First we have to find out, to which pin of the PWM board a motor is connected.
Look closely which motor moves and type in the answer.

Ensure to run the script until the end, otherwise your changes will not be saved!
This script can always be stopped with ctrl+c and restarted.
All other controls will be explained in the process.
###############
        '''
    )

    for pin_number in range(16):
        motor = Motor(pin_number, addr=address, busnum=busnum)
        motor.stop_motor()

    for pin_number in range(16):
        motor = Motor(pin_number, addr=address, busnum=busnum)
        motor.wiggle_motor()
        type_selection = ''
        while(1):
            print("Pin #{}".format(pin_number))
            print(
                'Was it a walking or PTU that moved, or should I repeat the movement? ')
            type_selection = input('(w)alking (p)tu (r)epeat - (n)one (f)inish_configuration\n')
            if(type_selection == 'w'):
                motor.pin_name += 'walk_'
                print('Good job\n')
                break
            elif(type_selection == 'p'):
                motor.pin_name += 'ptu_'
                print('Good job\n')
                break                
            elif(type_selection == 'r'):
                print('Look closely\n')
                motor.wiggle_motor()
            elif(type_selection == 'n'):
                print('Skipping pin')
                break
            elif(type_selection == 'f'):
                print('Finishing calibration at pin {}.'.format(pin_number))
                break
            else:
                print('Input must be w, p, r, n or f\n')
        
        if (type_selection == 'w' or type_selection == 'p'):
            while(1):
                print_exomy_layout()
                pos_selection = input(
                    'Type the position of the motor that moved.[1-8] or (r)epeat\n')
                if(pos_selection == 'r'):
                    print('Look closely\n')
                else:
                    try:
                        pos = int(pos_selection)
                        if(pos >= 1 and pos <= 8):
                            motor.pin_name += pos_names[pos]
                            print(f'{motor.pin_name}')
                            break
                        else:
                            print('The input was not a number between 1 and 8\n')
                    except ValueError:
                        print('The input was not a number between 1 and 8\n')
            
            pin_dict[motor.pin_name] = motor.pin_number
            print('Motor set!\n')
            print('########################################################\n')
        elif (type_selection == 'f'):
            break
    

    print('Now we will step through all the motors and check whether they have been assigned correctly.\n')
    print('Press ctrl+c if something is wrong and start the script again. \n')
    
    for pin_name in pin_dict:
        print('moving {}'.format(pin_name))
        print_exomy_layout()        
        
        pin = pin_dict[pin_name]
        motor = Motor(pin)
        motor.wiggle_motor()
        input('Press button to continue')
    
    print("You assigned {}/8 motors.".format(len(pin_dict.keys())))

    print('Write to config file.\n')
    update_config_file(pin_dict)
    print(
    '''
███████╗██╗███╗   ██╗██╗███████╗██╗  ██╗███████╗██████╗ 
██╔════╝██║████╗  ██║██║██╔════╝██║  ██║██╔════╝██╔══██╗
█████╗  ██║██╔██╗ ██║██║███████╗███████║█████╗  ██║  ██║
██╔══╝  ██║██║╚██╗██║██║╚════██║██╔══██║██╔══╝  ██║  ██║
██║     ██║██║ ╚████║██║███████║██║  ██║███████╗██████╔╝
╚═╝     ╚═╝╚═╝  ╚═══╝╚═╝╚══════╝╚═╝  ╚═╝╚══════╝╚═════╝
    ''')