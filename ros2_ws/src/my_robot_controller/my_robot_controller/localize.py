import math
import numpy as np
from robot_interfaces.msg import Direction


SIZE = 1.6
HALF_SIZE = SIZE / 2

NORMALS_FROM_CORNERS = {
    (Direction.NORTH_EAST, Direction.NORTH_WEST): np.array([0, -1]),
    (Direction.NORTH_WEST, Direction.NORTH_EAST): np.array([0, -1]),
    (Direction.NORTH_EAST, Direction.SOUTH_EAST): np.array([-1, 0]),
    (Direction.SOUTH_EAST, Direction.NORTH_EAST): np.array([-1, 0]),
    (Direction.NORTH_WEST, Direction.SOUTH_WEST): np.array([1, 0]),
    (Direction.SOUTH_WEST, Direction.NORTH_WEST): np.array([1, 0]),
    (Direction.SOUTH_EAST, Direction.SOUTH_WEST): np.array([0, 1]),
    (Direction.SOUTH_WEST, Direction.SOUTH_EAST): np.array([0, 1]),
}

POINT_FROM_CORNER = {
    Direction.NORTH_WEST: np.array([-HALF_SIZE, HALF_SIZE]),
    Direction.NORTH_EAST: np.array([HALF_SIZE, HALF_SIZE]),
    Direction.SOUTH_WEST: np.array([-HALF_SIZE, -HALF_SIZE]),
    Direction.SOUTH_EAST: np.array([HALF_SIZE, -HALF_SIZE]),
}

DIRECTIONS = [Direction.NORTH_EAST, Direction.NORTH_WEST, Direction.SOUTH_WEST, Direction.SOUTH_EAST]

# def normalize(v):
#     norm = np.linalg.norm(v)
#     if norm == 0:
#         return v
#     return v / norm

def positive_angle(theta: float) -> float:
    while (theta < 0):
        theta += 2 * np.pi
    return theta

def avg(lst):
    if len(lst) == 0:
        return None
    return sum(lst) / len(lst)


def third_point_of_the_circle(a: Direction, b: Direction, theta: float) -> np.array:
    if theta is None:
        return None

    n = NORMALS_FROM_CORNERS.get((a, b))
    bisector = HALF_SIZE * np.array([-n[0], -n[1]])
    h_ab = HALF_SIZE / np.tan(theta / 2)

    return bisector + h_ab * n


def circle_from_3_points(z1:complex, z2:complex, z3:complex) -> tuple[complex, float]:
    if (z1 == z2) or (z2 == z3) or (z3 == z1):
        raise ValueError(f"Duplicate points: {z1}, {z2}, {z3}")
        
    w = (z3 - z1)/(z2 - z1)
    
    # You should change 0 to a small tolerance for floating point comparisons
    if abs(w.imag) <= 0:
        raise ValueError(f"Points are collinear: {z1}, {z2}, {z3}")
        
    c = (z2 - z1)*(w - abs(w)**2)/(2j*w.imag) + z1  # Simplified denominator
    r = abs(z1 - c)
    
    return c, r


def circle_for_side(a: Direction, b: Direction, theta: float) -> tuple[complex, float]:
    third = third_point_of_the_circle(a, b, theta)
    c, r = circle_from_3_points(
        complex(*POINT_FROM_CORNER[a]),
        complex(*POINT_FROM_CORNER[b]),
        complex(*third)
    )
    return c, r


def get_intersections(x0, y0, r0, x1, y1, r1):
    # circle 1: (x0, y0), radius r0
    # circle 2: (x1, y1), radius r1

    d = math.sqrt((x1-x0)**2 + (y1-y0)**2)

    # non intersecting
    if d > r0 + r1:
        return None
    # One circle within other
    if d < abs(r0 - r1):
        return None
    # coincident circles
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d
        y2=y0+a*(y1-y0)/d
        x3=x2+h*(y1-y0)/d
        y3=y2-h*(x1-x0)/d

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d

        return (x3, y3, x4, y4)


def offset_loc(calib_data):
    NE, NW, SW, SE = [], [], [], []
    for angle, dir in calib_data:
        if dir == Direction.NORTH_EAST:
            NE.append(angle)
        elif dir == Direction.NORTH_WEST:
            NW.append(angle)
        elif dir == Direction.SOUTH_WEST:
            SW.append(angle)
        elif dir == Direction.SOUTH_EAST:
            SE.append(angle)

    mean_NE = avg(NE)
    mean_NW = avg(NW)
    mean_SW = avg(SW)
    mean_SE = avg(SE)

    corner_thetas = [mean_NE, mean_NW, mean_SW, mean_SE]
    valid = 4

    pos = corner_thetas.index(None)
    if pos != -1:
        valid = 3
        first = (pos + 1) % 4
    
    circles = []
    for q in range(valid - 1):
        i = (first + q) % 4
        j = (first + q + 1) % 4

        angle_diff = corner_thetas[j] - corner_thetas[i]
        if angle_diff < 0:
            angle_diff += 2 * np.pi
        circles.append(circle_for_side(DIRECTIONS[i], DIRECTIONS[j], angle_diff))
    
    intersections = []
    for i in range(len(circles) - 1):
        for j in range(i + 1, len(circles)):
            c1, r1 = circles[i]
            c2, r2 = circles[j]
            inter = get_intersections(c1.real, c1.imag, r1, c2.real, c2.imag, r2)
            if inter is not None:
                if abs(abs(inter[0]) + abs(inter[1]) - SIZE) > 0.01:    # select the intersection which is not one of the corners
                    intersections.append([inter[0], inter[1]])
                intersections.append([inter[2], inter[3]])

    x, y = np.mean(intersections, axis=0, dtype=np.float64)

    orientation = 0
    if mean_NE is not None:
        orientation += positive_angle(np.arctan2(HALF_SIZE - y, HALF_SIZE - x) - mean_NE)
    if mean_NW is not None:
        orientation += positive_angle(np.arctan2(HALF_SIZE - y, -HALF_SIZE - x) - mean_NW)
    if mean_SW is not None:
        orientation += positive_angle(np.arctan2(-HALF_SIZE - y, -HALF_SIZE - x) - mean_SW)
    if mean_SE is not None:
        orientation += positive_angle(np.arctan2(-HALF_SIZE - y, HALF_SIZE - x) - mean_SE)

    orientation /= 4 - corner_thetas.count(None)

    IR_SENSOR_OFFSET = 0.015
    x -= math.cos(orientation) * IR_SENSOR_OFFSET
    y -= math.sin(orientation) * IR_SENSOR_OFFSET 

    return [x, y, orientation]
