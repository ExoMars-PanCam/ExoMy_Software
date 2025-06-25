#!/usr/bin/env python3
import time
import rospy

from exomy.msg import MotorCommands
from standing_modes import StandingMode
from motors import Motors
from walking import Walking
from ptu import Ptu

ptu = Ptu()
motors = Motors()
walking = Walking()

# global watchdog_timer


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
    rospy.on_shutdown(shutdown)

    #global watchdog_timer
    #watchdog_timer = rospy.Timer(rospy.Duration(1.0), watchdog, oneshot=True)

    sub = rospy.Subscriber(
        "/motor_commands", MotorCommands, callback, queue_size=1)

    rate = rospy.Rate(10)

    rospy.spin()
