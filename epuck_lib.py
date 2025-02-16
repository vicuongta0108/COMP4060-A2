from math import *
import math
import numpy as np

# TODO:
#   Date 21 Jan 2025
#       - Define constants instead of hard-coded values                         : Done by Het on 21 Jan
#       - Measure the wheel diameter and wheel based and update it              :
#       - Research for how many bit the stepper motor uses for counting steps   :

# CONSTANTS
COUNTER_BITS = 32  # No. of bits used by stepper motor to store counter
WHEEL_DIAMETER = 41  # in mm
WHEEL_BASE_MM = 53
STEPS_PER_REVOLUTION = 1000

# steps_delta(last, current): int, calculates the difference in robot steps from the last
# position to the current, accounting for counter wraparound, and returns it as a signed integer.
# This one is tricky. Mine is 5 lines including def and return.
def steps_delta(last, current):
    # Step 1: Compute raw difference with wraparound
    delta = (current - last) % (2 ** COUNTER_BITS)  # Ensures diff is within [0, 2^32 - 1]

    # Step 2: Interpret as signed integer
    if delta >= 2 ** (COUNTER_BITS - 1):
        delta -= 2 ** COUNTER_BITS  # Convert to negative value

    return delta


# steps_to_rad(steps): float, converts signed motor steps to signed radians, and returns that
# value, using your knowledge of the motor construction. 2 lines
def steps_to_rad(steps):
    # (steps * 360 / 1000): Degree moved (1 revolution of motor have 1000 steps)
    # Then multiplying degree moved with (math.pi / 180) to get radians
    return float((steps * 360 / STEPS_PER_REVOLUTION) * (math.pi / 180))


# rad_to_steps(rad): float, converts signed radians to signed motor steps, and returns that
# value, using your knowledge of the motor construction. 2 lines
def rad_to_steps(rad):
    # rad * 180 / math.pi: Converting rad to degree
    # from degree figuring out # of steps by multiplying it by (1000 / 360)
    return float((rad * 180 / math.pi) * (STEPS_PER_REVOLUTION / 360))


# rad_to_mm(rad): float, converts the given signed radians of wheel rotation into expected
# signed ground distance, and returns that value. 2 lines
def rad_to_mm(rad):
    # Diameter of wheel is 41 mm
    # rad * 41 / 2: distance travelled in mm
    return float(rad * WHEEL_DIAMETER / 2)


# mm_to_rad(mm): float, converts the given mm distance into expected radians of wheel
# rotation, and returns that value. 2 lines
def mm_to_rad(mm):
    return float(mm * 2 / WHEEL_DIAMETER)


# steps_to_mm(steps): float, converts motor steps into expected ground distance, and returns
# that value. 2 lines
def steps_to_mm(steps):
    return float(steps * math.pi * WHEEL_DIAMETER / STEPS_PER_REVOLUTION)


# mm_to_steps(mm): float, converts expected ground distance into motor steps, and returns
# that value. 2 lines
def mm_to_steps(mm):
    return float(mm * STEPS_PER_REVOLUTION / math.pi / WHEEL_DIAMETER)


# print_pose ( (x_mm, y_mm, theta_rad) ), prints x,y,theta while converting theta to
# degrees. 3 lines
def print_pose(pos):
    # print(f'({pos[0]}mm, {pos[1]}mm, {pos[2] * 180 / math.pi}°)')
    # If for any reason the print_pose gives error then
    # uncomment the bottom line and comment the above line
    print(f'({round(pos[0])}, {round(pos[1])}, {round((pos[2] * 180 / math.pi) % 360)})')


def ret_pose(pos):
    # print(f'({pos[0]}mm, {pos[1]}mm, {pos[2] * 180 / math.pi}°)')
    # If for any reason the print_pose gives error then
    # uncomment the bottom line and comment the above line
    return pos[0], pos[1], pos[2] * 180 / math.pi


# print_pose((7, 7, math.pi))

# Debug trace
def debug_print(_print, script, msg):
    if _print: print(script + ": " + msg)

# Differential drive forward kinematic function from as1_task2
def calc_R(left_mm, right_mm):
    return (left_mm + right_mm) / (right_mm - left_mm) * WHEEL_BASE_MM / 2

def calc_icc_coordinates(rob_pos, r):
    x = rob_pos[0]
    y = rob_pos[1]
    theta = rob_pos[2]

    icc_x = x - (r * sin(theta))
    icc_y = y + (r * cos(theta))

    return (icc_x, icc_y)


def diff_linear_motion_kin(old_pos, distance_mm):
    x = old_pos[0]
    y = old_pos[1]
    theta = old_pos[2]

    new_x = x + distance_mm * cos(theta)
    new_y = y + distance_mm * sin(theta)

    return (new_x, new_y, theta)


def calc_omega(left_mm, right_mm):
    return (right_mm - left_mm) / WHEEL_BASE_MM


def get_rotation_matrix(omega):
    matrix = [[cos(omega), -sin(omega), 0],
              [sin(omega), cos(omega), 0],
              [0, 0, 1]]

    return np.array(matrix)


def get_icc_to_origin_vector(rob_pos, icc):
    vector = [[rob_pos[0] - icc[0]],  # x - icc_x
              [rob_pos[1] - icc[1]],  # y - icc_y
              [rob_pos[2]]]  # theta

    return np.array(vector)


def get_origin_to_icc_vector(icc, omega):
    vector = [[icc[0]],
              [icc[1]],
              [omega]]

    return np.array(vector)

def diff_drive_forward_kin(old_pos, left_steps, right_steps):
    # From old_pos figure ICC location
    # Then use the equation 5 from Computational Principles of Mobile Robotics
    # for calculating new position of robot
    left_mm = steps_to_mm(left_steps)
    right_mm = steps_to_mm(right_steps)

    if left_mm == right_mm:
        return diff_linear_motion_kin(old_pos, left_mm)

    r = calc_R(left_mm, right_mm)
    omega = calc_omega(left_mm, right_mm)

    icc_coor = calc_icc_coordinates(old_pos, r)

    rotation_matrix = get_rotation_matrix(omega)
    icc_to_orign_vec = get_icc_to_origin_vector(old_pos, icc_coor)
    origin_to_icc_vec = get_origin_to_icc_vector(icc_coor, omega)

    new_pos = (rotation_matrix.dot(icc_to_orign_vec)) + origin_to_icc_vec

    return (new_pos[0][0], new_pos[1][0], new_pos[2][0])

# Differential drive inverse kinematic function from as1_task4
def diff_drive_inverse_kin(distance_mm, speed_mm_s, omega_rad):
    # Stationary
    if speed_mm_s == 0:
        return 0, 0, 0, 0

    if distance_mm == 0:
        distance_mm = round(omega_rad * WHEEL_DIAMETER / 2)

        distance_steps = mm_to_steps(distance_mm)
        speed_steps_s = abs(mm_to_steps(speed_mm_s)) * (omega_rad / abs(omega_rad))

        return int(-1 * speed_steps_s), int(speed_steps_s), int(abs(distance_steps)), int(abs(distance_steps))

    time_to_travel = abs(distance_mm / speed_mm_s)  # in s
    turn_rate = omega_rad / time_to_travel  # omega w

    # Left and right wheel speeds in mm/s
    left_mm_s = speed_mm_s - turn_rate * (WHEEL_BASE_MM / 2)
    right_mm_s = speed_mm_s + turn_rate * (WHEEL_BASE_MM / 2)

    # Velocities in steps/s
    left_steps_s = mm_to_steps(left_mm_s)
    right_steps_s = mm_to_steps(right_mm_s)

    # Distance moved = velocities * time to travel
    left_steps = abs(left_steps_s * time_to_travel)
    right_steps = abs(right_steps_s * time_to_travel)

    return int(left_steps_s), int(right_steps_s), int(left_steps), int(right_steps)
