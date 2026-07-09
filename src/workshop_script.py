#!/usr/bin/env python3


import time


def _noop_request_image(con, sol, img):
    """Fallback when no image callback is provided."""
    return img


def run_workshop_script(motors, ptu, connection=None, image_fn=None, sol=1, img=1):
    """Placeholder workshop runner invoked from motor_node.

    Keep workshop sequencing logic in this file so motor_node stays lean.
    Sequence and command style are aligned with the workshop manual examples.
    """
    Request_image = image_fn if image_fn is not None else _noop_request_image

    con = connection

    # TODO: extend/replace this sequence as workshop scripting evolves.
    # PTU observation positions
    ptu.pan_transition(390)
    # img = Request_image(con,sol,img)

    ptu.pan_transition(300)
    # img = Request_image(con,sol,img)

    ptu.tilt_transition(410)
    # img = Request_image(con,sol,img)

    ptu.tilt_transition(340)
    ptu.pan_transition(240)
    # img = Request_image(con,sol,img)

    # Return PTU to default
    ptu.pan_transition(315)
    ptu.tilt_transition(340)

    # DRIVE commands
    motors.straight_drive(7, True)
    # img = Request_image(con,sol,img)

    # motors.straight_drive(5, False)
    # img = Request_image(con,sol,img)

    # SPOT TURN commands
    motors.point_turn(7, False)
    # img = Request_image(con,sol,img)

    # motors.point_turn(6.2, False)
    # img = Request_image(con,sol,img)

    # CRABBING command
    motors.crabbing_drive(-90, 5.0)

    motors.point_turn(7, False)
    motors.straight_drive(3, True)
    motors.point_turn(7, False)
    # img = Request_image(con,sol,img)
    time.sleep(60)

    return img
