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
from workshop_script import run_workshop_script
import socket

HOST = "172.17.0.1"  # Docker host IP on Linux (alternative to host.docker.internal)
PORT = 8015

ptu = Ptu()
motors = Motors()
walking = Walking()

# global watchdog_timer

def cleanup_and_exit(con=None, exit_code=0):
    """Perform cleanup operations and exit gracefully"""
    print("Performing cleanup...")
    try:
        if con is not None:
            try:
                con.close()
                print("Socket connection closed")
            except Exception as socket_error:
                print(f"Error closing socket: {socket_error}")

        # PTU back to default
        ptu.pan_transition(315)
        ptu.tilt_transition(340)

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

def setup_image_socket():
    # Connect to external echo server as a client
    try:
        print(f"Attempting to connect to echo server at {HOST}:{PORT}")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        print(f"Now Connecting")
        s.connect((HOST, PORT))
        print(f"Connected to echo server")
        return s
    except ConnectionRefusedError:
        print(f"Could not connect to echo server at {HOST}:{PORT}")
        print("Make sure echo_server.py is running outside Docker")
    except Exception as e:
        print(f"Client error: {e}")


def request_image(soc, sol, img):
    if soc is None:
        return img

    time.sleep(3)

    text = f"Sol_{sol:02d}_img_{img:02d}".encode('utf-8')
    img += 1
    soc.sendall(text)

    # Receive echo back from server
    response = soc.recv(1024)
    print(f"Received echo: {response.decode('utf-8')}")

    time.sleep(2)

    return img


if __name__ == "__main__":
    # This node waits for commands from the robot and sets the motors accordingly
    rospy.init_node("motors")
    rospy.loginfo("Starting the motors node")

    con = setup_image_socket()

    walking.stand() #! Only run once when first powering up

    sol = 1
    img = 1

    img = run_workshop_script(
        motors,
        ptu,
        connection=con,
        image_fn=request_image,
        sol=sol,
        img=img,
    )

    cleanup_and_exit(con)

    # rospy.on_shutdown(shutdown)

    #global watchdog_timer
    #watchdog_timer = rospy.Timer(rospy.Duration(1.0), watchdog, oneshot=True)


