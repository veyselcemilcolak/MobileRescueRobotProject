import math

RADIUS_ROBOT = 0.162
RADIUS_ROBOT = 0.15




def find_endpoints(slope, midpoint, length):

    # Calculate the direction vector using the slope
    direction_vector = (1, slope)

    # Normalize the direction vector
    magnitude = math.sqrt(direction_vector[0]**2 + direction_vector[1]**2)
    direction_vector = (direction_vector[0] / magnitude, direction_vector[1] / magnitude)

    # Scale the normalized direction vector by half the length of the line segment
    scaled_vector = (direction_vector[0] * length / 2, direction_vector[1] * length / 2)

    # Add and subtract the scaled vector from the midpoint to find the endpoints
    endpoint1 = (midpoint[0] + scaled_vector[0], midpoint[1] + scaled_vector[1])
    endpoint2 = (midpoint[0] - scaled_vector[0], midpoint[1] - scaled_vector[1])

    return endpoint1, endpoint2



def find_perpendicular_segments(p1, p2):

    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]

    # Calculate the slope of the given line segment
    if x2 - x1 != 0:
        m = (y2 - y1) / (x2 - x1)
    else:
        m = float('inf')  # vertical line, slope is infinity

    # Calculate the negative reciprocal of the slope to get the slope of a line perpendicular to the given line segment
    if m != 0:
        m_perpendicular = -1 / m
    else:
        m_perpendicular = 0  # horizontal line, slope is 0

    corner1, corner2 = find_endpoints(m_perpendicular, p1, 2*RADIUS_ROBOT)
    corner3, corner4 = find_endpoints(m_perpendicular, p2, 2*RADIUS_ROBOT)

    return [corner1, corner2, corner3, corner4]




def is_inside_rectangle(obstacle_point, start_point, end_point):

    x = obstacle_point[0]
    y = obstacle_point[1]
    point = (x, y)

    rectangle = find_perpendicular_segments(start_point, end_point)
    x1, y1 = rectangle[0]
    x2, y2 = rectangle[1]
    x3, y3 = rectangle[2]
    x4, y4 = rectangle[3]

    def is_inside_triangle(p1, p2, p3):
        # Function to check if a point is inside a triangle.
        def sign(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

        b1 = sign(point, p1, p2) < 0
        b2 = sign(point, p2, p3) < 0
        b3 = sign(point, p3, p1) < 0

        return ((b1 == b2) and (b2 == b3))

    # Check if the point is inside any of the triangles formed by the rectangle's corners.
    return is_inside_triangle((x1, y1), (x2, y2), (x3, y3)) or is_inside_triangle((x1, y1), (x3, y3), (x4, y4))





obstacle_point = [4, 2]
start_point = [5, 5]
end_point = [2, 2]


if is_inside_rectangle(obstacle_point, start_point, end_point):
    print("The point is inside the rectangle.")
else:
    print("The point is not inside the rectangle.")


# # Example usage:
# point = (3, 4)
# rectangle = [(1, 1), (4, 2), (5, 5), (2, 6)]

# if is_inside_rectangle(point, rectangle):
#     print(f"The point {point} is inside the rectangle.")
# else:
#     print(f"The point {point} is not inside the rectangle.")

# # Example usage:
# endpoint1 = (1, 2)
# endpoint2 = (5, 6)

# perpendicular_segments = find_perpendicular_segments(endpoint1, endpoint2)
# print("Endpoints of perpendicular segments:")
# for segment in perpendicular_segments:
#     print(segment)

# # Example usage:
# slope = 2  # Example slope
# midpoint = (3, 4)  # Example midpoint
# length = 4  # Example length

# endpoint1, endpoint2 = find_endpoints(slope, midpoint, length)
# print("Endpoints of the line segment:")
# print("Endpoint 1:", endpoint1)
# print("Endpoint 2:", endpoint2)