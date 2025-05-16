import numpy as np
import math


MIN_OBSTACLE_DIST = 0.25
OBSTACLE_FORCE = 1.0
MAX_SPEED = 1.0
TURNING_SPEED = 3.0

IR_SENSORS_ANGLE = [65.3, 38.0, 20.0, 3.0, -14.25, -34.0, -65.3]
IR_SENSORS = [[65.3,   1.1], [38.0,   1.2], [20.0,   1.0], [3.0,    1.0], [-14.25, 1.0], [-34.0,  1.2], [-65.3,  1.1]]
RADIUS_ROBOT = 0.162



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



# robot_loc = [x, y, theta], target_loc = [x, y], IR_obs = []

def ir_angle(frame_id):
    return IR_SENSORS[frame_id][0] / 180.0 * math.pi

def ir_weight(frame_id):
    return IR_SENSORS[frame_id][1]

class Vec2:
    def __init__(self, x, y) -> None:
        self.x = x
        self.y = y

    @classmethod

    def __add__(self, other):
        x = self.x + other.x
        y = self.y + other.y
        return Vec2(x, y)
    
    def __iadd__(self, other):
        self.x += other.x
        self.y += other.y
        return self
    
    def __sub__(self, other):
        x = self.x - other.x
        y = self.y - other.y
        return Vec2(x, y)
    
    def norm(self):
        return Vec2.dot(self, self)
    
    def cap_to_max_speed(self):
        length = self.norm()
        if length > MAX_SPEED:
            self.x *= MAX_SPEED / length
            self.y *= MAX_SPEED / length

    @classmethod
    def dot(cls, a, b):
        return math.sqrt(a.x * b.x + a.y * b.y)

    def orientation(self):
        return normalized_angle(math.atan2(self.y, self.x))
    
    def __str__(self):
        return "[x: {}, y: {}]".format(self.x, self.y)
    




def rotation_matrix(theta):
    rot_mat = np.array([[math.cos(theta), -math.sin(theta)],[math.sin(theta), math.cos(theta)]])
    return rot_mat

def coordinate_transformation(coord_body, base_loc, base_orient):
    coord_body = np.array(coord_body[0], coord_body[1])
    base_loc = np.array(base_loc[0], base_loc[1])
    rot_mat = rotation_matrix(base_orient)
    coord_world = rot_mat.dot(coord_body) + base_loc
    return [coord_world[0], coord_world[1]]

def obstacle_body_frame(ir_id, ir_value):

    dist =  -1 / 19 * math.log(ir_value / 3800)
    theta_obs = IR_SENSORS_ANGLE[ir_id] / 180.0 * math.pi
    x = dist * math.cos(theta_obs)
    y = dist * math.sin(theta_obs)

    return [x, y]
    

def obstacle_position(ir_id, ir_value, robot_loc):

    coord_body = obstacle_body_frame(ir_id, ir_value)
    obstacle_position = coordinate_transformation(coord_body, robot_loc[0: 2], robot_loc[2])

    return obstacle_position







def normalized_angle(theta: float):
    '''Normalize angle in radians to be between -pi and +pi'''
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi
    return theta    

def get_movement_direction(source: Vec2, target: Vec2):
    '''The angle under which the robot should move to get closer to the target'''
    return (target - source).orientation()

def force_from_obstacle(current_orientation, frame_id, value) -> Vec2:
    '''Calculate a force vector pointing in the opposite direction as the ir sensor is facing.
    The shorter the distance to the obstacle, the greater the magnitude of the force.'''
    # dist = -1 / 20 * math.log((value + 1) / 3800)
    dist = -1 / 19 * math.log(value / 3800)
    # node.get_logger().debug("{}: [{}, {}]".format(frame_id, value, dist))
    if dist >= MIN_OBSTACLE_DIST: # no obstacle, generate zero vector
        return Vec2(0, 0)
    
    theta = current_orientation + ir_angle(frame_id)
    force = ir_weight(frame_id) * OBSTACLE_FORCE / (dist * dist)
    x = -force * math.cos(theta)
    y = -force * math.sin(theta)
    return Vec2(x, y)

    

def vel_control(robot_loc, target_loc, IR_obs):
    
    if len(robot_loc) == 0 or len(target_loc) == 0 or len(IR_obs) == 0:
        return None
    

    move_vector = Vec2(target_loc[0] - robot_loc[0], target_loc[1] - robot_loc[1])    # distance vector between target and current position
    forces = Vec2(0, 0)

    for i in range(len(IR_obs)):
        forces += force_from_obstacle(robot_loc[2], i, IR_obs[i])

    move_vector += forces
    move_vector.cap_to_max_speed()

    target_orient = move_vector.orientation()        # the direction of the target from the current position
    delta_theta = normalized_angle(target_orient-robot_loc[2])  # the amount we should rotate to face the target

    rot_speed = TURNING_SPEED * delta_theta * min(1.0, max(0.2, 1 / (forces.norm() + 1e-5)))
    trans_speed = min(1.0, move_vector.norm())
        
    # don't move forward if the target is directly behind
    if abs(delta_theta) > 3 * math.pi / 4:
        trans_speed = 0.0

    return trans_speed, rot_speed



robot_loc = [0, 0, 0]
target_loc = [5, 0]
IR_obs = [0, 0, 0, 0, 20, 30, 40]
trans, rot = vel_control(robot_loc, target_loc, IR_obs)

print(trans, rot)
