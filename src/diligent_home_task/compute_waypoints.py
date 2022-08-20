#! /usr/bin/env python3
"""Compute waypoints library for polygonal paths."""
from math import pi, sin, cos
import numpy as np


def rot_matrix(theta):
    """Return a 2x2 rotation matrix for the given angle."""
    return np.array([
        [cos(theta), -sin(theta)],
        [sin(theta), cos(theta)],
        ])


def compute_waypoints(num_sides, length, origin=None):
    """
    Graded Diligent Robotics Function to produce a polygonal path.

    A 2D poses that form a regular polygon.
    An assumption is taken to generate the polygon taking the
    robot's current position as its circumcentre.

    Args:
        num_sides (int): The number of sides of the polygon.
        length (float): The length of each side of the polygon.
        origin (list): The current 2D pose of the robot.
    """
    if origin is None:
        origin = [0.0, 0.0, 0.0]

    waypoints = []

    # The angle required to rotate to each side of the polygon.
    exterior_angle = 2 * pi / num_sides

    # The angle required to rotate the robot to face the next waypoint.
    aligning_angle = (pi - exterior_angle) / 2

    # The centre of the polygon.
    centre = np.array(origin)

    for idx in range(num_sides):

        if idx == 0:
            # Creating a distance vector for the first waypoint.
            next_coord = length * np.array(
                [cos(origin[2]), sin(origin[2]), 0.0])

        else:
            aligning_angle += exterior_angle

            # Rotating the distance vector to face the next waypoint.
            next_coord[0:2] = rot_matrix(exterior_angle) @ next_coord[0:2]

            next_coord[2] = aligning_angle

        # Shift the origin to the centre of the polygon.
        waypoints.append((next_coord + centre).tolist())

    return waypoints
