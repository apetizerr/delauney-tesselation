"""
welzl.py
The program grabs the smallest circle using Welzl algorithm. The limitation would be the geometric calculation used
as the method to learn.
"""
from math import sqrt, hypot
from random import shuffle
from typing import Sequence

def get_euclidian_distance(a:float, b:float) -> float:
    """
    Get the Euclidean distance between two points.
    @param a: Tuple of two float coordinates (x, y).
    @param b: Tuple of two float coordinates (x, y).
    @return: Euclidean distance as a float.
    """
    a1, a2 = a
    b1, b2 = b
    return sqrt((a1 - b1)**2 + (a2 - b2)**2)


def get_midpoint(a:float, b:float)-> tuple:
    """
    Get the midpoint between two points in a tuple.
    @param a: Tuple with two coordinates as integers or floats.
    @param b: Tuple with two coordinates as integers or floats.
    @return: Midpoint in a tuple with two coordinates.
    """
    a1, a2 = a
    b1, b2 = b
    return (a1 + b1) / 2, (a2 + b2) / 2


def get_slope(a:float,b:float) -> float:
    """
    Get the slope of the line connecting two points.
    @param a: First point as a tuple.
    @param b: Second point as a tuple.
    @return: Slope as float or None if line is vertical.
    """
    a1, a2 = a
    b1, b2 = b
    if a2 == b2:
        return None
    return (b2 - a2) / (b1 - a1)


def get_perpendicular_slope(slope:float) -> float:
    """
    Get the slope of a line perpendicular to the given slope.
    @param slope: Slope of the original line.
    @return: Perpendicular slope or None.
    """
    if slope is None:
        return 0  # perpendicular to vertical: horizontal
    elif slope == 0:
        return None  # perpendicular to horizontal: vertical
    else:
        return -1 / slope


def get_intersection(slope1, slope2, intercept1, intercept2):
    """
    Find the intersection point of two lines given slopes and y-intercepts.
    @param slope1: Slope of the first line (or None if vertical).
    @param slope2: Slope of the second line (or None if vertical).
    @param intercept1: Intercept of the first line (y-intercept or x if vertical).
    @param intercept2: Intercept of the second line.
    @return: Tuple of intersection point or None if parallel.
    """
    if slope1 == slope2:
        if intercept1 == intercept2:
            return None
        else:
            return None
    x = (intercept2 - intercept1) / (slope1 - slope2)
    y = slope1 * x + intercept1
    return x, y


def get_circumcircle(a,b,c):
    """
    Compute the circumcircle (center and radius) passing through three points.
    @param a: First point as a tuple.
    @param b: Second point as a tuple.
    @param c: Third point as a tuple.
    @return: Tuple of (radius, center coordinates).
    """
    x1, y1 = get_midpoint(a,b)
    x2, y2 = get_midpoint(b,c)
    perp_slope_ab = get_perpendicular_slope(get_slope(a,b))
    perp_slope_bc = get_perpendicular_slope(get_slope(b,c))

    # y - y₁ = m(x - x₁)

    ab_intercept = (-perp_slope_ab * x1) + y1
    bc_intercept = (-perp_slope_bc * x2) + y2

    triangle_midpoint = get_intersection(slope1 = perp_slope_ab,
                                         slope2=perp_slope_bc,
                                         intercept1= ab_intercept,
                                         intercept2= bc_intercept)
    radius = get_euclidian_distance(triangle_midpoint, a)
    return radius, triangle_midpoint

def get_fast_circumcircle(points):
    # function aquired from wikipedia circumcircle cartesian coordinates
    (ax, ay), (bx, by), (cx, cy) = points

    d: float = (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)) * 2.0
    if d == 0.0:
        return None
    ux: float = ((ax ** 2 + ay ** 2) * (by - cy) + (bx**2 + by**2)
                 * (cy - ay) + (cx**2+cy**2) * (ay - by)) / d
    uy: float = ((ax ** 2 + ay ** 2) * (cx - bx) + (bx**2 + by**2)
                 * (ax - cx) + (cx**2+cy**2) * (bx - ax)) / d

    ra: float = hypot(ux - ax, uy - ay)
    rb: float = hypot(ux - bx, uy - by)
    rc: float = hypot(ux - cx, uy - cy)
    # returns the max radius to encompass all three points
    return (ux,uy) , max(ra, rb, rc)


def _handle_circle_points(r):
    """
    Create the smallest enclosing circle from 0 to 3 points.
    @param r: List of up to 3 points.
    @return: Tuple of (radius, center coordinates).
    """
    r_len = len(r)
    radius, center = None, None

    if r_len == 0:
        radius = 0
        center = (0, 0)
    elif r_len == 1:
        radius = 0
        center = (0, 0)
    elif r_len == 2:
        radius = get_euclidian_distance(r[0], r[1]) / 2
        center = get_midpoint(r[0], r[1])
    elif r_len == 3:
        coord1, coord2, coord3 = r
        radius, center = get_fast_circumcircle(a=coord1,b =coord2, c=coord3)
    else:
        raise ValueError("R should not contain more than 3 points")

    return radius, center


def get_welzl(P, R):
    """
    Recursively find the minimal enclosing circle for a set of points using Welzl's algorithm.
    @param P: List of input points to process.
    @param R: List of boundary points (must be on the circle).
    @return: Tuple of (radius, center coordinates).
    """
    if len(P) == 0 or len(R) == 3:
        return _handle_circle_points(R)

    p = P.pop()
    d = get_welzl(P.copy(), R.copy())
    radius, center = d

    if get_euclidian_distance(p, center) <= radius:
        return d
    else:
        return get_welzl(P.copy(), R + [p])


def get_smallest_circle(points: Sequence[tuple[float, float]]) -> tuple[float, tuple[float, float]]:
    """
    Compute the smallest enclosing circle for a set of points.
    @param points: Sequence of 2D points.
    @return: Tuple of (radius, center coordinates).
    """
    points = list(points)
    shuffle(points)
    return get_welzl(points, [])