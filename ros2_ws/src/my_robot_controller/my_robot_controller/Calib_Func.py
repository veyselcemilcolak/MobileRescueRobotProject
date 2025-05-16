from .FuncAndConst import *

def taking_average_of_3(array1, array2, array3, array4):

    average_of_3 = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]
    difference = [0, 0, 0, 0]
    for i in range(4):
        if i == 0:
            average_of_3[i][0] = (array2[0] + array3[0] + array4[0]) / 3
            average_of_3[i][1] = (array2[1] + array3[1] + array4[1]) / 3
            average_of_3[i][2] = (array2[2] + array3[2] + array4[2]) / 3
            difference[i] = math.sqrt((pow((average_of_3[i][0] - array1[0]),2))+(pow((average_of_3[i][1] - array1[1]),2))+(pow((average_of_3[i][2] - array1[2]),2)))
        elif i == 1:
            average_of_3[i][0] = (array1[0] + array3[0] + array4[0]) / 3
            average_of_3[i][1] = (array1[1] + array3[1] + array4[1]) / 3
            average_of_3[i][2] = (array1[2] + array3[2] + array4[2]) / 3
            difference[i] = math.sqrt((pow((average_of_3[i][0] - array2[0]),2))+(pow((average_of_3[i][1] - array2[1]),2))+(pow((average_of_3[i][2] - array2[2]),2)))
        elif i == 2:
            average_of_3[i][0] = (array2[0] + array1[0] + array4[0]) / 3
            average_of_3[i][1] = (array2[1] + array1[1] + array4[1]) / 3
            average_of_3[i][2] = (array2[2] + array1[2] + array4[2]) / 3
            difference[i] = math.sqrt((pow((average_of_3[i][0] - array3[0]),2))+(pow((average_of_3[i][1] - array3[1]),2))+(pow((average_of_3[i][2] - array3[2]),2)))
        else:
            average_of_3[i][0] = (array2[0] + array3[0] + array1[0]) / 3
            average_of_3[i][1] = (array2[1] + array3[1] + array1[1]) / 3
            average_of_3[i][2] = (array2[2] + array3[2] + array1[2]) / 3
            difference[i] = math.sqrt((pow((average_of_3[i][0] - array4[0]),2))+(pow((average_of_3[i][1] - array4[1]),2))+(pow((average_of_3[i][2] - array4[2]),2)))

    return average_of_3, difference

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
    
def order_IR(index_rm, average_angle, n):

    IR_array_order = []

    for i in range(3):
        instance = IR_Structure(0, 0, [0, 0])
        IR_array_order.append(instance)

    if n == 3:
        if index_rm == 0: # 2 4 3 -> 0 2 1
            IR_array_order[1].angle = average_angle[2]
            IR_array_order[0].angle = ClosestAngle(IR_array_order[1].angle, average_angle[0])
            IR_array_order[2].angle = ClosestAngle(IR_array_order[1].angle, average_angle[1])
            IR_array_order[0].direction = 2
            IR_array_order[1].direction = 4
            IR_array_order[2].direction = 3

        elif index_rm == 1: # 4 3 1 -> 2 1 0
            IR_array_order[1].angle = average_angle[1]
            IR_array_order[0].angle = ClosestAngle(IR_array_order[1].angle, average_angle[2])
            IR_array_order[2].angle = ClosestAngle(IR_array_order[1].angle, average_angle[0])
            IR_array_order[0].direction = 4
            IR_array_order[1].direction = 3
            IR_array_order[2].direction = 1

        elif index_rm == 2: # 1 2 4 -> 0 1 2
            IR_array_order[1].angle = average_angle[1]
            IR_array_order[0].angle = ClosestAngle(IR_array_order[1].angle, average_angle[0])
            IR_array_order[2].angle = ClosestAngle(IR_array_order[1].angle, average_angle[2])
            IR_array_order[0].direction = 1
            IR_array_order[1].direction = 2
            IR_array_order[2].direction = 4

        else: # 2 0 1
            IR_array_order[1].angle = average_angle[0]
            IR_array_order[0].angle = ClosestAngle(IR_array_order[1].angle, average_angle[2])
            IR_array_order[2].angle = ClosestAngle(IR_array_order[1].angle, average_angle[1])
            IR_array_order[0].direction = 3
            IR_array_order[1].direction = 1
            IR_array_order[2].direction = 2

    elif n == 4:
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
    direc_order = [1, 2, 3, 4] # four directions, one will be deleted

    mul_detec_num = [0, 0, 0, 0] # for multi_detection case, number of detection for each corner
    first_angle = [0, 0, 0, 0]
    sum_angle = [0, 0, 0, 0] # sum the angle values, for taking average later
    num_detected = 0 # number of detected corners

    average_angle = [0, 0, 0]
    average_angle4 = [0, 0, 0, 0]
    index_rm = 0
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


    if num_detected == 3:
        index_rm = mul_detec_num.index(0)
        del mul_detec_num[index_rm]
        del sum_angle[index_rm]
        del direc_order[index_rm]

        for i in range(3):
            average_angle[i] = sum_angle[i] / mul_detec_num[i]
            average_angle[i] = mPI2pPI(average_angle[i])        

    elif num_detected == 4:
        for i in range(4):
            average_angle4[i] = sum_angle[i] / mul_detec_num[i]
            average_angle4[i] = mPI2pPI(average_angle4[i])

        for i in range(4):
            if i == 0:
                angle_order1[1] = average_angle4[3]
                angle_order1[0] = ClosestAngle(angle_order1[1], average_angle4[1])
                angle_order1[2] = ClosestAngle(angle_order1[1], average_angle4[2])
            elif i == 1:
                angle_order2[1] = average_angle4[2]
                angle_order2[0] = ClosestAngle(angle_order2[1], average_angle4[3])
                angle_order2[2] = ClosestAngle(angle_order2[1], average_angle4[0])
            elif i == 2:
                angle_order3[1] = average_angle4[1]
                angle_order3[0] = ClosestAngle(angle_order3[1], average_angle4[0])
                angle_order3[2] = ClosestAngle(angle_order3[1], average_angle4[3])
            else:
                angle_order4[1] = average_angle4[0]
                angle_order4[0] = ClosestAngle(angle_order4[1], average_angle4[2])
                angle_order4[2] = ClosestAngle(angle_order4[1], average_angle4[1])

    else:
        logging.info("Insufficient Data, Calibration Failed!")
        return None
    
    if num_detected == 3:
        IR_array_order3 = order_IR(index_rm, average_angle, 4)
    elif num_detected == 4:
        IR_array_order41 = order_IR(0, angle_order1, 4)
        IR_array_order42 = order_IR(1, angle_order2, 4)
        IR_array_order43 = order_IR(2, angle_order3, 4)
        IR_array_order44 = order_IR(3, angle_order4, 4)
        


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


    if num_detected == 3:
        offset_pos, offset_angle = offset_equation(IR_array_order3)
    elif num_detected == 4:
        offset_pos41, offset_angle41 = offset_equation(IR_array_order41)
        offset_angle41 = mPI2pPI(offset_angle41)

        offset_pos42, offset_angle42 = offset_equation(IR_array_order42)
        offset_angle42 = mPI2pPI(offset_angle42)

        offset_pos43, offset_angle43 = offset_equation(IR_array_order43)
        offset_angle43 = mPI2pPI(offset_angle43)

        offset_pos44, offset_angle44 = offset_equation(IR_array_order44)
        offset_angle44 = mPI2pPI(offset_angle44)

        print(offset_pos41, offset_angle41)
        print(offset_pos42, offset_angle42)
        print(offset_pos43, offset_angle43)
        print(offset_pos44, offset_angle44)

        offset_ave_3, diff = taking_average_of_3([offset_pos41[0], offset_pos41[1], offset_angle41], [offset_pos42[0], offset_pos42[1], offset_angle42], [offset_pos43[0], offset_pos43[1], offset_angle43], [offset_pos44[0], offset_pos44[1], offset_angle44])

        ind_max = np.argmax(diff)
        offset_pos[0] = offset_ave_3[ind_max][0]
        offset_pos[1] = offset_ave_3[ind_max][1]
        offset_angle = offset_ave_3[ind_max][2]

        # offset_pos[0] = (offset_pos41[0] + offset_pos42[0] + offset_pos43[0] + offset_pos44[0]) / 4
        # offset_pos[1] = (offset_pos41[1] + offset_pos42[1] + offset_pos43[1] + offset_pos44[1]) / 4
        # offset_angle = (offset_angle41 + offset_angle42 + offset_angle43 + offset_angle44) / 4




    offset[0] = float(offset_pos[0])
    offset[1] = float(offset_pos[1])
    offset[2] = float(mPI2pPI(offset_angle))

    if offset[0] < -dis_IR_rec/2 or offset[0] > dis_IR_rec/2 or offset[1] < -dis_IR_rec/2 or offset[1] > dis_IR_rec/2:
        logging.info("Got Impossible Position on Field, Calibration Failed!")
        return None
    
    print(f"position before IR offset: {offset}")

    IR_SENSOR_OFFSET = 0.015
    offset[0] -= math.cos(offset[2]) * IR_SENSOR_OFFSET
    offset[1] -= math.sin(offset[2]) * IR_SENSOR_OFFSET 
    
    print(f"position aftr IR offset: {offset}")

    return offset
