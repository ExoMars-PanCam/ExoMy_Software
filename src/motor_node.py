#!/usr/bin/env python3
import time
import rospy
import os
import signal
import sys

from exomy.msg import MotorCommands
from standing_modes import StandingMode
from motors import Motors
from walking import Walking
from ptu import Ptu
import socket

HOST = "172.17.0.1"  # Docker host IP on Linux (alternative to host.docker.internal)
PORT = 8015


ptu = Ptu()
motors = Motors()
walking = Walking()

# global watchdog_timer

def cleanup_and_exit(exit_code=0):
    """Perform cleanup operations and exit gracefully"""
    print("Performing cleanup...")
    try:
        # Stop all motors
        motors.stopMotors()
        motors.setDriving([0, 0, 0, 0, 0, 0])
        print("Motors stopped")
        
        # Put robot in safe position
        # walking.sit()  # Uncomment if you want to sit the robot
        print("Robot in safe position")
        
        # Clean shutdown of ROS node if it's running
        if rospy.is_shutdown() == False:
            rospy.signal_shutdown("Clean exit requested")
            print("ROS node shutdown")
            
    except Exception as e:
        print(f"Error during cleanup: {e}")
    
    print("Cleanup complete. Exiting...")
    sys.exit(exit_code)


def callback(cmds):
    motors.setSteering(cmds.motor_angles)
    motors.setDriving(cmds.motor_speeds)
    ptu.setPanTilt(cmds.ptu_angles)

    if cmds.standing_mode == StandingMode.SIT.value:
        rospy.loginfo(f"Now going to sit be careful!!!!")
        walking.sit()
    elif cmds.standing_mode == StandingMode.STAND.value:
        rospy.loginfo(f"Now going to stand be careful!!!!")
        walking.stand()

    # global watchdog_timer
    # watchdog_timer.shutdown()
    # # If this timer runs longer than the duration specified,
    # # then watchdog() is called stopping the driving motors.
    # # watchdog_timer = rospy.Timer(rospy.Duration(5.0), watchdog, oneshot=True)


def shutdown():
    motors.stopMotors()
    rospy.loginfo("Running this command")
    # walking.sit()


def watchdog(event):
    rospy.loginfo("Watchdog fired. Stopping driving motors.")
    # motors.stopMotors()
    # walking.sit()


if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")

    ## ---------------------------------------------------------------------------------------------
    ## Start of scripting

    ptu.wake()



    ## ---------------------------------------------------------------------------------------------
    ## End of Script 
    cleanup_and_exit()

    # rospy.on_shutdown(shutdown)

    #global watchdog_timer
    #watchdog_timer = rospy.Timer(rospy.Duration(1.0), watchdog, oneshot=True)


