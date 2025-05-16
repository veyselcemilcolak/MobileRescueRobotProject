import numpy as np
import math

from .FuncAndConst import eps_ANG, eps_LIN_PICK, eps_LIN_LEAVE, R_GRIPPER

MIN_OBSTACLE_DIST = 0.25
OBSTACLE_FORCE = 20.0
MAX_SPEED = 0.15
TURNING_SPEED = 1.
TOLERANCE = 0.03

RADIUS_ROBOT = 0.1
# orientation and weight of the individual ir sensors
IR_SENSORS = [[65.3,   0.4], [38.0,   0.8], [20.0,   1.0], [3.0,    1.0], [-14.25, 1.0], [-34.0,  0.8], [-65.3,  0.4]]
IR_SENSORS_ANGLE = [65.3, 38.0, 20.0, 3.0, -14.25, -34.0, -65.3]




# robot_loc = [x, y, theta], target_loc = [x, y], IR_obs = []

def rotation_matrix(theta):
    rot_mat = [math.cos(theta), -math.sin(theta), math.sin(theta), math.cos(theta)]
    return rot_mat

def coordinate_transformation(coord_body, base_loc, base_orient):
    coord_world = [0, 0]
    rot_mat = rotation_matrix(base_orient)
    coord_world[0] = rot_mat[0]*coord_body[0] + rot_mat[1]*coord_body[1] + base_loc[0]
    coord_world[1] = rot_mat[2]*coord_body[0] + rot_mat[3]*coord_body[1] + base_loc[1]
    return coord_world

def obstacle_body_frame(ir_id, ir_value):

    dist =  -1 / 20 * math.log((ir_value+1) / 3800)
    theta_obs = IR_SENSORS_ANGLE[ir_id] / 180.0 * math.pi
    x = dist * math.cos(theta_obs)
    y = dist * math.sin(theta_obs)

    return [x, y]
    

def obstacle_position(ir_id, ir_value, robot_loc):

    coord_body = obstacle_body_frame(ir_id, ir_value)
    obstacle_position = coordinate_transformation(coord_body, robot_loc[0: 2], robot_loc[2])

    return obstacle_position






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

    def __mul__(self, other):
        x = other * self.x
        y = other * self.y
        return Vec2(x, y)
    
    __rmul__ = __mul__
    
    def norm(self):
        return Vec2.dot(self, self)
    
    def normalized(self) -> 'Vec2':
        length = self.norm()
        return Vec2(self.x / length, self.y / length)
    
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
    dist = -1 / 20 * math.log((value + 1) / 3800)
    # node.get_logger().debug("{}: [{}, {}]".format(frame_id, value, dist))
    if dist >= MIN_OBSTACLE_DIST: # no obstacle, generate zero vector
        return Vec2(0, 0)
    # if value < 20:
    #     return Vec2(0, 0)
    
    theta = current_orientation + ir_angle(frame_id)
    force = ir_weight(frame_id) * OBSTACLE_FORCE * value / (dist * dist + 1e-5)
    x = -force * math.cos(theta)
    y = -force * math.sin(theta)
    return Vec2(x, y)

    

def vel_control(robot_loc: Vec2,
                robot_theta: float,
                IR_obs = [],
                distance_eps: float = eps_LIN_LEAVE,
                gripper_target: Vec2 = None,
                robot_target: Vec2 = None,
                obsacle_avoidance: bool = True):
    if gripper_target is None and robot_target is None:
        raise ValueError("gripper_target or robot_target must be set")
    
    if gripper_target and robot_target:
        print(f"gripper: {gripper_target}")
        print(f"robot: {robot_target}")
        raise ValueError("Only one of gripper_target or robot_target can be set")

    if not robot_loc:
        raise ValueError("robot_loc is empty")
    
    if robot_target:
        target_loc = robot_target
        offset = 0.0
    if gripper_target:
        target_loc = gripper_target
        offset = R_GRIPPER
    
    robot_orient = normalized_angle(robot_theta)  # conversion between orientation systems

    move_vector = target_loc - robot_loc        # distance vector between final and current position
    dist_target = abs(move_vector.norm() - offset)   # scalar distance
    print(f"dist_target: {dist_target}")

    forces = Vec2(0, 0)
    if obsacle_avoidance and not vel_control.still_rotating:
        for i in range(len(IR_obs)):
            # obstacle_point = obstacle_position(i, IR_obs[i], robot_loc)
            # if IR_obs[i] > 55:
            #     if is_inside_rectangle(obstacle_point, robot_loc, target_loc):
            #         forces += force_from_obstacle(robot_loc[2], i, IR_obs[i])
            if IR_obs[i] < 200.0:
                continue
            forces += force_from_obstacle(robot_orient, i, IR_obs[i])

        print(f"move_vector: {move_vector}, forces: {forces}")

        FORCE_FACTOR = max(min(1.0, (dist_target - 0.2) * 5),0.0) ## niklas: added max such that it cant change polarity
        move_vector += FORCE_FACTOR * forces

    # if max(IR_obs) > 200.0:
    #     current_turning_speed = TURNING_SPEED
    # else:
    #     current_turning_speed = TURNING_SPEED/2

    # move_vector.cap_to_max_speed()

    target_orient = move_vector.orientation()        # the direction of the target from the current position
    delta_theta = normalized_angle(target_orient - robot_orient)  # the amount we should rotate to face the target
    print(f"move_vector with forces: {move_vector}, force_factor: FORCE_FACTOR")
    print(f"target_orient: {target_orient},\trobot_orient: {robot_orient},\tdelta_theta: {delta_theta}")
    
    rot_speed = TURNING_SPEED * min(1.0, max(-1.0, delta_theta)) * min(1, 10 * dist_target + 0.02) #* min(1.0, max(0.2, 1 / (move_vector.norm() + 1e-5)))
    trans_speed = min(MAX_SPEED, max(5 * (dist_target + 0.05) * (move_vector.norm() - offset + 0.05), 0.02))
    
    # if abs(rot_speed) > 0.9:
    #     sign = 1 if vel_control.prev_rot_speed >= 0 else 0
    #     rot_speed = sign * abs(rot_speed)

    if vel_control.still_rotating and abs(delta_theta) < eps_ANG * 3:
        vel_control.still_rotating = False

    # don't move forward if the target is directly behind or have arrived
    if (abs(delta_theta) > 3 * math.pi / 4) or (dist_target < distance_eps) \
            or (abs(delta_theta) > math.pi / 8 and dist_target < 3 * distance_eps) \
            or vel_control.still_rotating:
        trans_speed = 0.0

    if dist_target < distance_eps:
        sign = 1 if rot_speed >= 0 else -1
        rot_speed = sign * 0.05

    # gripper_current = Vec2(robot_loc.x + R_GRIPPER * math.cos(robot_orient), robot_loc.y + R_GRIPPER * math.sin(robot_orient))
    # if (gripper_current - gripper_target).norm() < 0.05:
    if dist_target < distance_eps and (not gripper_target or abs(delta_theta) < eps_ANG):
        ready = True
        trans_speed = 0.0
        rot_speed = 0.0
        vel_control.still_rotating = True
    else:
        ready = False

    # vel_control.prev_rot_speed = rot_speed

    print(f"linear: {trans_speed}, angular: {rot_speed}, ready: {ready}")
    return trans_speed, rot_speed, ready


# intialize static variable
# vel_control.prev_rot_speed = 0
vel_control.still_rotating = True
