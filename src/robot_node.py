#!/usr/bin/env python3
import time
from exomy.msg import RoverCommand, MotorCommands, Screen
from standing_modes import StandingMode
import rospy
from rover import Rover
import message_filters


global exomy
exomy = Rover()
standing_mode = StandingMode.UNKNOWN.value


def joy_callback(message):
    global standing_mode
    cmds = MotorCommands()

    cmds.standing_mode = message.standing_mode

    if message.motors_enabled is True:
        exomy.setLocomotionMode(message.locomotion_mode)

        cmds.motor_angles = exomy.joystickToSteeringAngle(
            message.vel, message.steering)
        cmds.motor_speeds = exomy.joystickToVelocity(
            message.vel, message.steering)
        cmds.ptu_angles = self.robot.joystickToPTUAngle(
            msg.pan, msg.tilt, msg.ptu_reset)
    else:
        cmds.motor_angles = exomy.joystickToSteeringAngle(0, 0)
        cmds.motor_speeds = exomy.joystickToVelocity(0, 0)
        cmds.standing_mode = message.standing_mode

    robot_pub.publish(cmds)


if __name__ == '__main__':
    rospy.init_node('robot_node')
    rospy.loginfo("Starting the robot node")
    global robot_pub
    joy_sub = rospy.Subscriber(
        "/rover_command", RoverCommand, joy_callback, queue_size=1)

    rate = rospy.Rate(10)

    robot_pub = rospy.Publisher("/motor_commands", MotorCommands, queue_size=1)

    rospy.spin()
