# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       prane                                                        #
# 	Created:      9/27/2026, 11:05:24 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)

# Left side motors
left_motor_a = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
left_motor_b = Motor(Ports.PORT2, GearSetting.RATIO_6_1, False)

# Right side motors
right_motor_a = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
right_motor_b = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)

# Motor groups so each side moves together
left_drive = MotorGroup(left_motor_a, left_motor_b)
right_drive = MotorGroup(right_motor_a, right_motor_b)

# Deadband to prevent motor drift from stick noise near center
DEADBAND = 5

def apply_deadband(value):
    if abs(value) < DEADBAND:
        return 0
    return value

# Main control loop
while True:
    # Left stick Y-axis (axis3) = forward/back
    forward = apply_deadband(controller_1.axis3.position())
    # Right stick X-axis (axis1) = turning
    turn = apply_deadband(controller_1.axis1.position())

    # Arcade drive mixing
    left_speed = forward + turn
    right_speed = forward - turn

    # Clamp values to valid percentage range
    left_speed = max(-100, min(100, left_speed))
    right_speed = max(-100, min(100, right_speed))

    left_drive.set_velocity(left_speed, PERCENT)
    right_drive.set_velocity(right_speed, PERCENT)

    left_drive.spin(FORWARD)
    right_drive.spin(FORWARD)

    wait(20, MSEC)