# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records.
#
# 02_b_filter_motor_file
# Claus Brenner, 09 NOV 2012
from math import sin, cos, pi
from lego_robot import *

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).


def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):

    l = motor_ticks[0] * ticks_to_mm
    r = motor_ticks[1] * ticks_to_mm

    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.

        # --->>> compute x, y, theta as:
        # theta = old_theta
        # x = old_x + L*cos(theta)
        # y = old_y + L*sin(theta)
        theta = old_pose[2]
        x = old_pose[0] + l * cos(theta)
        y = old_pose[1] + l * sin(theta)

        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.
        theta = old_pose[2]
        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        x_pose = old_pose[0] - scanner_displacement * cos(theta)
        y_pose = old_pose[1] - scanner_displacement * sin(theta)
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        # alpha = (r-l)/w
        # R = l/alpha
        alpha = (motor_ticks[1] - motor_ticks[0]) * ticks_to_mm / robot_width
        R = motor_ticks[0] * ticks_to_mm/alpha

        # x_center = old_x - (R+(w/2))*sin(theta)
        # y_center = old_y + (R+(w/2))*cos(theta)
        x_center = x_pose - (R + robot_width/2) * sin(theta)
        y_center = y_pose + (R + robot_width/2) * cos(theta)

        # --->>> compute x, y, theta here.
        # theta = old_theta + alpha
        # x = x_center - (R+(w/2))*sin(theta)
        # y = y_center + (R+(w/2))*cos(theta)
        theta = (theta + alpha) % (2 * pi)
        x = x_center + (R + robot_width/2) * sin(theta)
        y = y_center - (R + robot_width/2) * cos(theta)

        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        x = x + scanner_displacement * cos(theta)
        y = y + scanner_displacement * sin(theta)

        return (x, y, theta)


if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Measured start position.
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print("F %f %f %f" % pose, file=f)

    f.close()
