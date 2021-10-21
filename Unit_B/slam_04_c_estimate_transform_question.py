# For each cylinder in the scan, find its cartesian coordinates,
# in the world coordinate system.
# Find the closest pairs of cylinders from the scanner and cylinders
# from the reference, and the optimal transformation which aligns them.
# Then, output the scanned cylinders, using this transform.
# 04_c_estimate_transform
# Claus Brenner, 14 NOV 2012
from lego_robot import *
from slam_b_library import filter_step
from slam_04_a_project_landmarks import\
    compute_scanner_cylinders, write_cylinders
from math import sqrt

# Given a list of cylinders (points) and reference_cylinders:
# For every cylinder, find the closest reference_cylinder and add
# the index pair (i, j), where i is the index of the cylinder, and
# j is the index of the reference_cylinder, to the result list.
# This is the function developed in slam_04_b_find_cylinder_pairs.


def find_cylinder_pairs(cylinders, reference_cylinders, max_radius):
    cylinder_pairs = []

    for i, cyl in enumerate(cylinders):
        x, y = cyl

        closest_cyl = None
        best_dis_squared = max_radius * max_radius

        for j, ref in enumerate(reference_cylinders):
            dx, dy = ref[0] - x, ref[1] - y
            distance_squared = dx * dx + dy * dy
            if distance_squared < best_dis_squared:
                best_dis_squared = distance_squared
                closest_cyl = j

        if closest_cyl:
            cylinder_pairs.append((i, closest_cyl))

    return cylinder_pairs

# Given a point list, return the center of mass.


def compute_center(point_list):
    # Safeguard against empty list.
    if not point_list:
        return (0.0, 0.0)
    # If not empty, sum up and divide.
    sx = sum([p[0] for p in point_list])
    sy = sum([p[1] for p in point_list])
    return (float(sx) / len(point_list), float(sy) / len(point_list))

# Given a left_list of points and a right_list of points, compute
# the parameters of a similarity transform: scale, rotation, translation.
# If fix_scale is True, use the fixed scale of 1.0.
# The returned value is a tuple of:
# (scale, cos(angle), sin(angle), x_translation, y_translation)
# i.e., the rotation angle is not given in radians, but rather in terms
# of the cosine and sine.


def estimate_transform(left_list, right_list, fix_scale=False):
    # Compute left and right center.
    lc = compute_center(left_list)
    rc = compute_center(right_list)

    r_prime = [(r[0] - rc[0], r[1] - rc[1]) for r in right_list]
    l_prime = [(l[0] - lc[0], l[1] - lc[1]) for l in left_list]
    # --->>> Insert your previous solution here.
    ll = 0.
    rr = 0.
    cs = 0.
    ss = 0.
    for i in range(len(l_prime)):
        l, r = l_prime[i], r_prime[i]
        ll += (l[0]**2 + l[1]**2)
        rr += (r[0]**2 + r[1]**2)
        cs += r[0]*l[0] + r[1]*l[1]
        ss += -r[0]*l[1] + r[1]*l[0]
    if ll == 0. or rr == 0.:
        return None

    if fix_scale:
        la = 1
    else:
        la = sqrt(rr/ll)

    norm = sqrt(cs**2 + ss**2)
    c = cs/norm
    s = ss/norm
    tx = rc[0] - la * (lc[0] * c - lc[1] * s)
    ty = rc[1] - la * (lc[0] * s + lc[1] * c)

    return la, c, s, tx, ty

# Given a similarity transformation:
# trafo = (scale, cos(angle), sin(angle), x_translation, y_translation)
# and a point p = (x, y), return the transformed point.


def apply_transform(trafo, p):
    la, c, s, tx, ty = trafo
    lac = la * c
    las = la * s
    x = lac * p[0] - las * p[1] + tx
    y = las * p[0] + lac * p[1] + ty
    return (x, y)


if __name__ == '__main__':
    # The constants we used for the filter_step.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 150.0

    # The constants we used for the cylinder detection in our scan.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0

    # The maximum distance allowed for cylinder assignment.
    max_cylinder_distance = 300.0

    # The start pose we obtained miraculously.
    pose = (1850.0, 1897.0, 3.717551306747922)

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("Unit_B/robot4_motors.txt")
    logfile.read("Unit_B/robot4_scan.txt")

    # Also read the reference cylinders (this is our map).
    logfile.read("Unit_B/robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    out_file = open("Unit_B/estimate_transform.txt", "w")
    for i in range(len(logfile.scan_data)):
        # Compute the new pose.
        pose = filter_step(pose, logfile.motor_ticks[i],
                           ticks_to_mm, robot_width,
                           scanner_displacement)

        # Extract cylinders, also convert them to world coordinates.
        cartesian_cylinders = compute_scanner_cylinders(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset)
        world_cylinders = [LegoLogfile.scanner_to_world(pose, c)
                           for c in cartesian_cylinders]

        # For every cylinder, find the closest reference cylinder.
        cylinder_pairs = find_cylinder_pairs(
            world_cylinders, reference_cylinders, max_cylinder_distance)

        # Estimate a transformation using the cylinder pairs.
        trafo = estimate_transform(
            [world_cylinders[pair[0]] for pair in cylinder_pairs],
            [reference_cylinders[pair[1]] for pair in cylinder_pairs],
            fix_scale=True)

        # Transform the cylinders using the estimated transform.
        transformed_world_cylinders = []
        if trafo:
            transformed_world_cylinders =\
                [apply_transform(trafo, c) for c in
                 [world_cylinders[pair[0]] for pair in cylinder_pairs]]

        # Write to file.
        # The pose.
        print("F %f %f %f" % pose, file=out_file)
        # The detected cylinders in the scanner's coordinate system.
        write_cylinders(out_file, "D C", cartesian_cylinders)
        # The detected cylinders, transformed using the estimated trafo.
        write_cylinders(out_file, "W C", transformed_world_cylinders)

    out_file.close()
