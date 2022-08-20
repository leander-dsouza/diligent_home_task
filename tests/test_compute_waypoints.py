#! /usr/bin/env python3
"""Script to analyse the edge cases of the waypoints library."""
import unittest
from math import sin, cos, pi
from diligent_home_task.compute_waypoints import compute_waypoints

bash_colours = {
    'red': '\033[1;31m',
    'green': '\033[1;32m',
    'reset': '\033[0m'
}


class TestComputeWaypoints(unittest.TestCase):
    """Class fo testing the compute_waypoints() function."""

    def test_waypoints_length(self):
        """
        Test the length of the waypoints list.

        Test whether the waypoints are equal to the sides of the polygon.
        """
        num_sides = 5
        length = 3.0
        waypoints = compute_waypoints(num_sides, length)

        if len(waypoints) != num_sides:
            self.fail(
                bash_colours['red'] +
                "The number of waypoints is not equal" +
                "to the number of sides." +
                bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The number of waypoints is equal" +
            "to the number of sides." +
            bash_colours['reset'])

    def test_polygonal_sides(self):
        """
        Vary the number of sides of the polygon.

        Test the generation of sides of the polygon
        keeping the length and origin same as the first test.
        """
        num_sides = 3
        length = 3.0
        waypoints = [
            waypoint[0:2] for waypoint in
            compute_waypoints(num_sides, length)]
        result = [
            [length * cos(0), length * sin(0)],
            [length * cos(2*pi/3), length * sin(2*pi/3)],
            [length * cos(4*pi/3), length * sin(4*pi/3)],
            ]

        for waypoint_coord, result_coord in zip(waypoints, result):

            self.assertAlmostEqual(
                waypoint_coord[0], result_coord[0],
                msg=bash_colours['red'] +
                f"X coordinate {waypoint_coord[0]} is not equal" +
                "to the expected value " +
                f"{result_coord[0]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[1], result_coord[1],
                msg=bash_colours['red'] +
                f"Y coordinate {waypoint_coord[1]} is not equal" +
                "to the expected value " +
                f"{result_coord[1]}" + bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The variation of polygonal sides match" +
            "with the expected output." +
            bash_colours['reset'])

    def test_polygonal_length(self):
        """
        Vary the length of the polygon.

        Test the varying the length of each side of the polygon
        keeping the number of sides and origin same as the first test.
        """
        num_sides = 5
        length = 6.0
        waypoints = [
            waypoint[0:2] for waypoint in
            compute_waypoints(num_sides, length)]
        result = [
            [length * cos(0), length * sin(0)],
            [length * cos(2*pi/5), length * sin(2*pi/5)],
            [length * cos(4*pi/5), length * sin(4*pi/5)],
            [length * cos(6*pi/5), length * sin(6*pi/5)],
            [length * cos(8*pi/5), length * sin(8*pi/5)],
            ]

        for waypoint_coord, result_coord in zip(waypoints, result):

            self.assertAlmostEqual(
                waypoint_coord[0], result_coord[0],
                msg=bash_colours['red'] +
                f"X coordinate {waypoint_coord[0]} is not equal" +
                "to the expected value " +
                f"{result_coord[0]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[1], result_coord[1],
                msg=bash_colours['red'] +
                f"Y coordinate {waypoint_coord[1]} is not equal"
                + "to the expected value " +
                f"{result_coord[1]}" + bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The variation of polygonal length match" +
            "with the expected output." +
            bash_colours['reset'])

    def test_polygonal_follow(self):
        """
        Test the following angle of the polygon.

        Test the robot heading angle needed to follow the
        polygon starting at its centre invoking the least turns
        keeping the number of length, sides and origin same as the first test.
        """
        num_sides = 5
        length = 3.0
        headings = [
            heading[-1] for heading in
            compute_waypoints(num_sides, length)]
        result = [
            0.0,
            3*pi/10 + 2*pi/5,
            3*pi/10 + 4*pi/5,
            3*pi/10 + 6*pi/5,
            3*pi/10 + 8*pi/5,
            ]

        for heading_coord, result_coord in zip(headings, result):

            self.assertAlmostEqual(
                heading_coord, result_coord,
                msg=bash_colours['red'] +
                f"Theta coordinate {heading_coord} is not equal" +
                "to the expected value " +
                f"{result_coord}" + bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The follow polygon heading test has passed." +
            bash_colours['reset'])

    def test_polygonal_origin(self):
        """
        Vary the origin of the polygon.

        Test the varying the origin of the polygon
        keeping the number of sides and length same as the first test.
        """
        num_sides = 5
        length = 3.0
        origin = [1.0, 2.0, pi/4]
        waypoints = compute_waypoints(num_sides, length, origin)
        result = [
            [length * cos(0+origin[2]), length * sin(0+origin[2]),
                0.0],
            [length * cos(2*pi/5+origin[2]), length * sin(2*pi/5+origin[2]),
                3*pi/10 + 2*pi/5],
            [length * cos(4*pi/5+origin[2]), length * sin(4*pi/5+origin[2]),
                3*pi/10 + 4*pi/5],
            [length * cos(6*pi/5+origin[2]), length * sin(6*pi/5+origin[2]),
                3*pi/10 + 6*pi/5],
            [length * cos(8*pi/5+origin[2]), length * sin(8*pi/5+origin[2]),
                3*pi/10 + 8*pi/5],
            ]

        for waypoint_coord, result_coord in zip(waypoints, result):

            self.assertAlmostEqual(
                waypoint_coord[0], result_coord[0] + origin[0],
                msg=bash_colours['red'] +
                f"X coordinate {waypoint_coord[0]} is not equal" +
                "to the expected value " +
                f"{result_coord[0] + origin[0]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[1], result_coord[1] + origin[1],
                msg=bash_colours['red'] +
                f"Y coordinate {waypoint_coord[1]} is not equal" +
                "to the expected value " +
                f"{result_coord[1] + origin[1]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[2], result_coord[2] + origin[2],
                msg=bash_colours['red'] +
                f"Theta coordinate {waypoint_coord[2]} is not equal" +
                "to the expected value " +
                f"{result_coord[2] + origin[2]}" + bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The variation of polygonal origin match " +
            "with the expected output." +
            bash_colours['reset'])

    def test_arbitrary_polygon(self):
        """Test the varying all the features of the polygon."""
        num_sides = 4
        length = 7.0
        origin = [5.0, 1.0, pi/2]
        waypoints = compute_waypoints(num_sides, length, origin)
        result = [
            [length * cos(0+origin[2]), length * sin(0+origin[2]),
                0.0],
            [length * cos(2*pi/4+origin[2]), length * sin(2*pi/4+origin[2]),
                pi/4 + 2*pi/4],
            [length * cos(4*pi/4+origin[2]), length * sin(4*pi/4+origin[2]),
                pi/4 + 4*pi/4],
            [length * cos(6*pi/4+origin[2]), length * sin(6*pi/4+origin[2]),
                pi/4 + 6*pi/4],
            ]

        for waypoint_coord, result_coord in zip(waypoints, result):

            self.assertAlmostEqual(
                waypoint_coord[0], result_coord[0] + origin[0],
                msg=bash_colours['red'] +
                f"X coordinate {waypoint_coord[0]} is not equal" +
                "to the expected value " +
                f"{result_coord[0] + origin[0]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[1], result_coord[1] + origin[1],
                msg=bash_colours['red'] +
                f"Y coordinate {waypoint_coord[1]} is not equal" +
                "to the expected value " +
                f"{result_coord[1] + origin[1]}" + bash_colours['reset'])

            self.assertAlmostEqual(
                waypoint_coord[2], result_coord[2] + origin[2],
                msg=bash_colours['red'] +
                f"Theta coordinate {waypoint_coord[2]} is not equal" +
                "to the expected value " +
                f"{result_coord[2] + origin[2]}" + bash_colours['reset'])

        print(
            bash_colours['green'] +
            "The variation of polygonal origin, length, number of sides and" +
            "heading matches with the expected output." +
            bash_colours['reset'])


if __name__ == "__main__":
    unittest.main()
