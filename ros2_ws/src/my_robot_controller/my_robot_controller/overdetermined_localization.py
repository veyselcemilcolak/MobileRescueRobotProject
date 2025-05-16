from .FuncAndConst import *


def correct_angles(robot_location):
    angles = [0, 0, 0, 0]
    for i in range(4):
        angles[i] = math.atan2((Corners_Location[i][1] - robot_location[1]),(Corners_Location[i][0] - robot_location[0])) - robot_location[2]
        # angles[i] = angles[i]/math.pi*180
    return angles


class IR_Structure:
    def __init__(self, direction, angle, coordinate):
        self.direction = direction
        self.angle = angle
        self.coordinate = coordinate
        
        
def mPI2pPI(angle):
    if -math.pi <= angle <= math.pi:
        return angle
    elif angle > math.pi:
        return angle - 2*math.pi
    else:
        return angle + 2*math.pi
    

def ClosestAngle(angle1, angle2):
    res1 = abs(angle1 - angle2)
    res2 = abs(angle1 - (angle2 + 2*math.pi))
    res3 = abs(angle1 - (angle2 - 2*math.pi))
    res_index = np.argmin([res1, res2, res3])
    if res_index == 0:
        return angle2
    elif res_index ==1:
        return angle2 + 2*math.pi
    else:
        return angle2 - 2*math.pi
    
def order_IR(index_rm, average_angle):

    IR_array_order = []

    for i in range(3):
        instance = IR_Structure(0, 0, [0, 0])
        IR_array_order.append(instance)


    if index_rm == 0: # 2 4 3 -> 0 2 1
        IR_array_order[1].angle = average_angle[1]
        IR_array_order[0].angle = average_angle[0]
        IR_array_order[2].angle = average_angle[2]
        IR_array_order[0].direction = 2
        IR_array_order[1].direction = 4
        IR_array_order[2].direction = 3

    elif index_rm == 1: # 4 3 1 -> 2 1 0
        IR_array_order[1].angle = average_angle[1]
        IR_array_order[0].angle = average_angle[0]
        IR_array_order[2].angle = average_angle[2]
        IR_array_order[0].direction = 4
        IR_array_order[1].direction = 3
        IR_array_order[2].direction = 1

    elif index_rm == 2: # 1 2 4 -> 0 1 2
        IR_array_order[1].angle = average_angle[1]
        IR_array_order[0].angle = average_angle[0]
        IR_array_order[2].angle = average_angle[2]
        IR_array_order[0].direction = 1
        IR_array_order[1].direction = 2
        IR_array_order[2].direction = 4

    else: # 2 0 1
        IR_array_order[1].angle = average_angle[1]
        IR_array_order[0].angle = average_angle[0]
        IR_array_order[2].angle = average_angle[2]
        IR_array_order[0].direction = 3
        IR_array_order[1].direction = 1
        IR_array_order[2].direction = 2


    for i, instance in enumerate(IR_array_order):
        if IR_array_order[i].direction == 2:
            IR_array_order[i].coordinate = [dis_IR_rec/2, dis_IR_rec/2]
        elif IR_array_order[i].direction == 4:
            IR_array_order[i].coordinate = [dis_IR_rec/2, -dis_IR_rec/2]
        elif IR_array_order[i].direction == 1:
            IR_array_order[i].coordinate = [-dis_IR_rec/2, dis_IR_rec/2]
        else:
            IR_array_order[i].coordinate = [-dis_IR_rec/2, -dis_IR_rec/2]

    return IR_array_order


    

def offset_loc(calib_data):

    length_data = len(calib_data)

    mul_detec_num = [0, 0, 0, 0] # for multi_detection case, number of detection for each corner
    first_angle = [0, 0, 0, 0]
    sum_angle = [0, 0, 0, 0] # sum the angle values, for taking average later
    num_detected = 0 # number of detected corners

    average_angle = [0, 0, 0, 0]
    angle_order1 = [0, 0, 0]
    angle_order2 = [0, 0, 0]
    angle_order3 = [0, 0, 0]
    angle_order4 = [0, 0, 0]
    offset_pos = [0, 0]
    offset_angle = 0


    offset = [0, 0, 0]


    for i in range(length_data):
        if calib_data[i][1] == 1:
            mul_detec_num[0] = mul_detec_num[0] + 1
            if mul_detec_num[0] == 1:
                first_angle[0] = calib_data[i][0]
            elif mul_detec_num[0] > 1:
                calib_data[i][0] = ClosestAngle(first_angle[0],calib_data[i][0])
            sum_angle[0] = sum_angle[0] + calib_data[i][0]
        elif calib_data[i][1] == 2:
            mul_detec_num[1] = mul_detec_num[1] + 1
            if mul_detec_num[1] == 1:
                first_angle[1] = calib_data[i][0]
            elif mul_detec_num[1] > 1:
                calib_data[i][0] = ClosestAngle(first_angle[1],calib_data[i][0])
            sum_angle[1] = sum_angle[1] + calib_data[i][0]
        elif calib_data[i][1] == 3:
            mul_detec_num[2] = mul_detec_num[2] + 1
            if mul_detec_num[2] == 1:
                first_angle[2] = calib_data[i][0]
            elif mul_detec_num[2] > 1:
                calib_data[i][0] = ClosestAngle(first_angle[2],calib_data[i][0])
            sum_angle[2] = sum_angle[2] + calib_data[i][0]
        elif calib_data[i][1] == 4:
            mul_detec_num[3] = mul_detec_num[3] + 1
            if mul_detec_num[3] == 1:
                first_angle[3] = calib_data[i][0]
            elif mul_detec_num[3] > 1:
                calib_data[i][0] = ClosestAngle(first_angle[3],calib_data[i][0])
            sum_angle[3] = sum_angle[3] + calib_data[i][0]


    num_detected = len(np.nonzero(mul_detec_num)[0])
   

    if num_detected == 4:
        for i in range(4):
            average_angle[i] = sum_angle[i] / mul_detec_num[i]
            average_angle[i] = mPI2pPI(average_angle[i])

        for i in range(4):
            if i == 0:
                angle_order1[1] = average_angle[3]
                angle_order1[0] = ClosestAngle(angle_order1[1], average_angle[1])
                angle_order1[2] = ClosestAngle(angle_order1[1], average_angle[2])
            elif i == 1:
                angle_order2[1] = average_angle[2]
                angle_order2[0] = ClosestAngle(angle_order2[1], average_angle[3])
                angle_order2[2] = ClosestAngle(angle_order2[1], average_angle[0])
            elif i == 2:
                angle_order3[1] = average_angle[1]
                angle_order3[0] = ClosestAngle(angle_order3[1], average_angle[0])
                angle_order3[2] = ClosestAngle(angle_order3[1], average_angle[3])
            else:
                angle_order4[1] = average_angle[0]
                angle_order4[0] = ClosestAngle(angle_order4[1], average_angle[2])
                angle_order4[2] = ClosestAngle(angle_order4[1], average_angle[1])

    else:
        # logging.info("Insufficient Data, Calibration Failed!")
        return None
    
    IR_array_order41 = order_IR(0, angle_order1)
    IR_array_order42 = order_IR(1, angle_order2)
    IR_array_order43 = order_IR(2, angle_order3)
    IR_array_order44 = order_IR(3, angle_order4)
        


    def offset_equation(IR_array_order):

        # angle between the three connecting lines
        alpha1 = IR_array_order[0].angle - IR_array_order[1].angle
        alpha2 = IR_array_order[1].angle - IR_array_order[2].angle


        def Eq1(x):

            return (math.sin(math.pi/2 - alpha2 + x)/math.sin(alpha2))-(math.sin(math.pi - alpha1 - x)/math.sin(alpha1))


        beta = fsolve(Eq1, math.pi/4)[0]

        

        # calculate length of triangle for x, y with the IR in the middle
        gamma = math.pi - alpha1 - beta


        def Eq2(x):

            return (dis_IR_rec-x)*math.tan(gamma) - x*math.tan(beta)
        
        
        x_dis = fsolve(Eq2, dis_IR_rec/2)[0]

        

        y_dis = x_dis * math.tan(beta)



        offset_pos = [0, 0]

        if IR_array_order[1].direction == 2:
            offset_pos = np.add(IR_array_order[1].coordinate, [-x_dis, -y_dis])
        elif IR_array_order[1].direction == 4:
            offset_pos = np.add(IR_array_order[1].coordinate, [-y_dis, x_dis])
        elif IR_array_order[1].direction == 1:
            offset_pos = np.add(IR_array_order[1].coordinate, [y_dis, -x_dis])
        else:
            offset_pos = np.add(IR_array_order[1].coordinate, [x_dis, y_dis])


        # orientation
        omega = 0
        offset_angle = 0


        if IR_array_order[1].direction == 2:
            omega = math.atan(abs(IR_array_order[1].coordinate[1] - offset_pos[1])/abs(IR_array_order[1].coordinate[0] - offset_pos[0]))
            offset_angle = omega - IR_array_order[1].angle
        elif IR_array_order[1].direction == 4:
            omega = math.atan(abs(IR_array_order[1].coordinate[0] - offset_pos[0])/abs(IR_array_order[1].coordinate[1]- offset_pos[1]))
            offset_angle = omega - IR_array_order[1].angle + 3*math.pi/2
        elif IR_array_order[1].direction == 1:
            omega = math.atan(abs(IR_array_order[1].coordinate[0] - offset_pos[0])/abs(IR_array_order[1].coordinate[1] - offset_pos[1]))
            offset_angle = omega - IR_array_order[1].angle + math.pi/2
        else:
            omega = math.atan(abs(IR_array_order[1].coordinate[1] - offset_pos[1])/abs(IR_array_order[1].coordinate[0] - offset_pos[0]))
            offset_angle = omega - IR_array_order[1].angle + math.pi

        return offset_pos, offset_angle


    offset_pos41, offset_angle41 = offset_equation(IR_array_order41)
    offset_angle41 = mPI2pPI(offset_angle41)

    offset_pos42, offset_angle42 = offset_equation(IR_array_order42)
    offset_angle42 = mPI2pPI(offset_angle42)

    offset_pos43, offset_angle43 = offset_equation(IR_array_order43)
    offset_angle43 = mPI2pPI(offset_angle43)

    offset_pos44, offset_angle44 = offset_equation(IR_array_order44)
    offset_angle44 = mPI2pPI(offset_angle44)


    offset_pos[0] = (offset_pos41[0] + offset_pos42[0] + offset_pos43[0] + offset_pos44[0]) / 4
    offset_pos[1] = (offset_pos41[1] + offset_pos42[1] + offset_pos43[1] + offset_pos44[1]) / 4
    offset_angle = (offset_angle41 + offset_angle42 + offset_angle43 + offset_angle44) / 4



    offset[0] = float(offset_pos[0])
    offset[1] = float(offset_pos[1])
    offset[2] = float(mPI2pPI(offset_angle))


    print(f"position before overditermined: {offset}")

    angles_cal =  correct_angles(offset)


    for i in range(4):
        average_angle[i] = ClosestAngle(angles_cal[i], average_angle[i])


    print(f"calculated angles: {angles_cal}")
    print(f"measured angles: {average_angle}")


    def function_to_minimize(x):

        return ((math.atan2((Corners_Location[0][1] - x[1]),(Corners_Location[0][0] - x[0])) - x[2]) - average_angle[0]) ** 2 + ((math.atan2((Corners_Location[1][1] - x[1]),(Corners_Location[1][0] - x[0])) - x[2]) - average_angle[1]) ** 2 + ((math.atan2((Corners_Location[2][1] - x[1]),(Corners_Location[2][0] - x[0])) - x[2]) - average_angle[2]) ** 2 + ((math.atan2((Corners_Location[3][1] - x[1]),(Corners_Location[3][0] - x[0])) - x[2]) - average_angle[3]) ** 2
    

    # Initial guess
    initial_guess = offset

    # Minimize the function
    result = minimize(function_to_minimize, initial_guess).x


    if result[0] < -dis_IR_rec/2 or result[0] > dis_IR_rec/2 or result[1] < -dis_IR_rec/2 or result[1] > dis_IR_rec/2:
        # logging.info("Got Impossible Position on Field, Calibration Failed!")
        return None
    
    print(f"position before IR offset: {result}")

    IR_SENSOR_OFFSET = 0.015
    result[0] -= math.cos(result[2]) * IR_SENSOR_OFFSET
    result[1] -= math.sin(result[2]) * IR_SENSOR_OFFSET 
    
    print(f"position aftr IR offset: {result}")

    return list(result)


