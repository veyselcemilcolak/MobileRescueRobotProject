from .FuncAndConst import *
from .Calib_Func import offset_loc
from .localize import offset_loc as lazars_offset_loc
from .Vel_Control import vel_control, Vec2
from .overdetermined_localization import offset_loc as over_det

namespace = ""




class ControllerNode (Node):

    def __init__(self):
        super().__init__(node_name="control_node")
        self.cmd_vel_publisher_ = self.create_publisher(
        Twist, "/cmd_vel", 10)
        self.servo_publisher_ = self.create_publisher(
        MoveServo, "/move_servo", 10)
        self.led_publisher_ = self.create_publisher(
        Bool, "/led", 10)
        self.grip_publisher_ = self.create_publisher(
        Empty, "/gripper_sensors", 10)
        self.localize_publisher_  = self.create_publisher(
        Int32, "/localize3", 10)
        self.state_publisher_ = self.create_publisher(
        State, "/state_r", 10)
        self.offset_publisher = self.create_publisher(
        Float32MultiArray, "/offset", 10)
        
        self.input_subscriber = self.create_subscription(
            FinalInput, "/fine_input", self.input_callback, 10)

        self.get_logger ().info("Robot controller has been started.")
        self.state = "init"
        self.cmd = Float32MultiArray()
        self.cmdl = Int32()
        self.cmdled = Bool()
        self.cmdser = MoveServo()
        self.cmdv = Twist()
        self.cmdst = State()
        self.cmdg = Empty()
        self.led_state = "off"
        self.arm_state = "down"
        self.gripper_state = "open"
        self.led_angle = 0
        self.calib_angle_servo = 0
        self.client = self.create_client(SetParameters, namespace + "/motion_control/set_parameters")
        self.set_params()
    
    def set_params(self):
        request = SetParameters.Request()
        param = Parameter()
        param.name = "safety_override"
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = "full"
        request.parameters.append(param)
        self.client.wait_for_service()
        self.future = self.client.call_async(request)

    
    def servo (self, ser_id, angle ):
        angle = float(angle)
        self.cmdser.id =ser_id
        self.cmdser.angle=angle
        self.servo_publisher_.publish (self.cmdser)
        if ser_id == MoveServo.SERVO_ARM:
            if angle == ARM_SERVO_UP_ANGLE:
                self.arm_state = "up"
            if angle == ARM_SERVO_DOWN_ANGLE:
                self.arm_state = "down"
            if angle == ARM_SERVO_CALIB_ANGLE:
                self.arm_state = "calibration mode"
        if ser_id == MoveServo.SERVO_GRIPPER:
            if angle == GRIPPER_SERVO_CLOSING_ANGLE:
                self.gripper_state = "closed"
            if angle == GRIPPER_SERVO_OPENING_ANGLE:
                self.gripper_state = "open"
        if ser_id == MoveServo.SERVO_LED:
            self.led_angle = (angle + LED_SERVO_OFFSET)%360
    
    
    def led(self, led_in):
        self.cmdled.data = led_in
        self.led_publisher_.publish (self.cmdled)
        if led_in:
            self.led_state = "on"
        else:
            self.led_state = "off"

    def full_loc(self, inp):
        self.cmdl.data = inp
        self.localize_publisher_.publish (self.cmdl)     

    def vel(self, lin, rot):
        self.cmdv.linear.x = lin
        self.cmdv.angular.z = rot
        self.cmd_vel_publisher_.publish (self.cmdv)

    def offs(self, val):
        self.cmd.data = val
        self.offset_publisher.publish(self.cmd)

    def stategen(self):
        self.cmdst.state = self.state
        self.cmdst.arm_state = self.arm_state
        self.cmdst.gripper_state = self.gripper_state
        self.cmdst.led_state = self.led_state
        self.cmdst.x = self.x
        self.cmdst.y = self.y
        self.cmdst.theta = self.theta
        self.cmdst.led_angle = self.led_angle
        self.state_publisher_.publish(self.cmdst)

        
    def input_callback(self, inp: FinalInput):
        
        self.sensor_values = inp.sensor_values
        self.x = inp.x
        self.y = inp.y 
        self.theta = inp.theta
        self.command_type = inp.command_type
        self.command_values = inp.command_values
        self.command_flag = inp.command_flag
        self.calib_dir  = inp.calib_dir
        self.calib_angle_read = inp.calib_angle
        self.gripper_full = inp.gripper_full

        if self.command_flag:
            if self.command_type == Command.LED_ON: # If there is a command to turn LED on for (x,y)
                # Get the rotation angle for xLED,yLED from current location
                
                
                _, trg =  target_nav(self.x + LED_R*np.cos(LED_TH + self.theta), 
                                     self.y + LED_R*np.sin(LED_TH + self.theta),
                                     self.theta,
                                     self.command_values[0],
                                     self.command_values[1])
                ang = (int(180*trg/PI) - LED_SERVO_OFFSET)%360
                
                if ang >175:
                    self.state = "LED turn"
                    logging.info(self.state)
                    self.led_target = [self.command_values[0],self.command_values[1]]
                else:
                    self.led(True)
                    self.servo(MoveServo.SERVO_LED, ang)
    
            elif self.command_type ==Command.LED_OFF:
                self.led(False)

            elif self.command_type == Command.GIVESTATE:
                self.stategen()
        
        if self.state == "init":
            logging.info(self.state)
            self.state = "idle"
            logging.info(self.state)
            self.calib_array = []
            self.all_calib_angles = []
            self.run_target0 = None
            self.run_target1 = None
            self.run_target = None
            self.calibrate_again = False
            self.calib_rot = -1
            self.move_time = None
            self.led(False)
            self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)
            self.servo(MoveServo.SERVO_GRIPPER, GRIPPER_SERVO_OPENING_ANGLE)
            self.servo(MoveServo.SERVO_LED, (-LED_SERVO_OFFSET)%360)
            self.n_down = 0
            self.only_pick =False
            self.angle_target = None

            #self.servo(4, CALIB_SERVO_VEL_ZERO)
            self.n_tour =0


        
        if self.state =="idle":

            #logging.info(self.state)

            if (self.command_flag and self.command_type ==Command.CALIBRATE) or self.calibrate_again:
                self.calibrate_again=False
                self.state = "calibrate"
                logging.info(self.state)
                self.calib_rot *=-1
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_CALIB_ANGLE)
                if self.calib_rot>0:
                    self.full_loc(1)  
                else:
                    self.full_loc(2) 
                

            if  self.command_flag and self.command_type ==Command.MAIN_OPERATION:
                self.state = "run2pick"
                logging.info(self.state)
                self.run_target0 = self.command_values[0:2]
                self.run_target1 = self.command_values[2:4]
                self.run_target = self.run_target0


            if self.command_flag and self.command_type ==Command.GOTO:
                self.state = "goto"
                logging.info(self.state)
                self.run_target = self.command_values

            if self.command_flag and self.command_type ==Command.PICKUP:
                self.state = "pickup1"
                logging.info(self.state)

            if self.command_flag and self.command_type ==Command.PUTDOWN:
                self.state = "putdown1"
                logging.info(self.state)

            if self.command_flag and self.command_type ==Command.TURNTO:
                self.state = "turnto"
                logging.info(self.state)
                self.turn_target = self.command_values

            if self.command_flag and self.command_type ==Command.GOSTR:
                self.state = "go str"
                logging.info(self.state)
                self.move_time = self.command_values[0]/FINE_MOVEMENT_SPEED

            if self.command_flag and self.command_type ==Command.TURN:
                self.state = "turn_rad"
                logging.info(self.state)
                self.angle_target = (self.command_values[0] + self.theta)%(2*PI)

            if self.command_flag and self.command_type ==Command.LEAVETO:
                self.state = "run2deliver"
                logging.info(self.state)
                self.run_target = self.command_values

            if self.command_flag and self.command_type ==Command.PICKFROM:
                self.state = "run2pick"
                logging.info(self.state)
                self.run_target = self.command_values
                self.only_pick =True
        
        if self.state =="calibrate":
            #logging.info(self.state)
            for n in range(len(self.calib_dir)):
                self.all_calib_angles.append(self.calib_angle_read[n]%(2*PI))
                if self.calib_dir[n]>Direction.NONE:
                    self.calib_array.append([(self.calib_angle_read[n] + CALIB_SERVO_OFFSET*PI/180)%(2*PI), self.calib_dir[n]])
            #logging.info(self.all_calib_angles)
            if finished(self.all_calib_angles):
                self.n_tour +=1
                logging.info(self.n_tour)
                self.full_loc(0)
                self.state = "idle"
                logging.info(self.state)
                self.all_calib_angles = []
                if self.n_tour<N_CALIB_TOUR:
                    self.calibrate_again = True
                else:
                    self.n_tour =0
                    logging.info(self.calib_array)
                    #loc_res = lazars_offset_loc(self.calib_array)
                    #logging.info(f"Lazar's algorithm: {loc_res}")
                    loc_res = offset_loc(self.calib_array)
                    logging.info(f"Original algorithm: {loc_res}")
                    loc_res = over_det(self.calib_array)
                    logging.info(f"Overdetermined algorithm: {loc_res}")
                    self.calib_array = []
                    if loc_res is None:
                        logging.info("Calibration FAILED! Restart Calibration!")
                        self.calibrate_again = True
                    else:
                        self.offs(loc_res)
                        self.calibrate_again = False
                        self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)


        if self.state == "run2pick":
            #logging.info(self.state)
            print(f"self.theta: {self.theta}")

            move_lin, move_ang, ready = vel_control(Vec2(self.x, self.y),
                                                    self.theta,
                                                    self.sensor_values,
                                                    eps_LIN_PICK,
                                                    gripper_target=Vec2(self.run_target[0], self.run_target[1]))
            
            if ready:
                move_lin, move_ang = 0.0,0.0
                self.state = "armdown1"
                self.n_down = -1
                logging.info(self.state)
            self.vel(move_lin, move_ang)

        if self.state == "armdown1":
            #logging.info(self.state)
            if timing(ARM_DOWN_TIME):
                self.state ="move1"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_DOWN_ANGLE)
        
        if self.state == "move1":
            #logging.info(self.state)
            if timing(SMALL_MOVEMENT_TIME):
                self.state ="wait gripper message 1"
                logging.info(self.state)
                self.grip_publisher_.publish(self.cmdg)
                self.vel(0.0, 0.0)
            else:
                self.vel(FINE_MOVEMENT_SPEED, 0.0)
        
        if self.state =="wait gripper message 1":
            if timing(2):
                self.n_down +=1
                if self.n_down == len(TURN_ANGLE_LIST):
                    self.state = "idle"
                    logging.info("I cant find object.")
                    self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)
                    logging.info(self.state)
                    self.n_down = -1
                else:
                    self.state = "armdownup1"
                    logging.info(self.state)
            else:
                if self.gripper_full:
                    self.n_down = -1
                    self.state = "grasp"
                    logging.info(self.state)

        if self.state == "armdownup1":
            if timing(ARM_UP_TIME):
                self.state = "moveback1"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)

        if self.state =="moveback1":
            if timing(SMALL_MOVEMENT_TIME):
                self.state = "turnarrange1"
                logging.info(self.state)
                self.vel(0.0, 0.0)
            else:
                self.vel(-FINE_MOVEMENT_SPEED, 0.0)
                

        if self.state == "turnarrange1":
            if timing(abs(TURN_ANGLE_LIST[self.n_down])/W_FINE):
                self.state = "armdown1"
                logging.info(self.state)
                self.vel(0.0, 0.0)
            else:
                self.vel(0.0, W_FINE*np.sign(TURN_ANGLE_LIST[self.n_down]))
        
        
        
        if self.state == "grasp":
            #logging.info(self.state)
            if timing(GRASP_TIME):
                self.state = "armup1"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_GRIPPER, GRIPPER_SERVO_CLOSING_ANGLE)
        

        if self.state == "armup1":
            #logging.info(self.state)
            if timing(ARM_UP_TIME):
                if self.only_pick:
                    self.run_target = None
                    self.state = "idle"
                    self.only_pick = False
                else:
                    self.run_target = self.run_target1
                    self.state = "run2deliver"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)
        

        if self.state == "run2deliver":
            move_lin, move_ang, ready = vel_control(Vec2(self.x, self.y),
                                                    self.theta,
                                                    self.sensor_values,
                                                    eps_LIN_LEAVE,
                                                    gripper_target=Vec2(self.run_target[0], self.run_target[1]))
            if ready:
                move_lin, move_ang = 0.0,0.0
                self.state = "armdown2"
                logging.info(self.state)
                     
            self.vel(move_lin, move_ang)

        if self.state == "armdown2":
            #logging.info(self.state)
            if timing(ARM_DOWN_TIME):
                self.state = "release"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_DOWN_ANGLE)

        if self.state == "release":
            #logging.info(self.state)
            if timing(RELEASE_TIME):
                self.state = "small_reverse"
                logging.info(self.state)  
            else:
                self.servo(MoveServo.SERVO_GRIPPER, GRIPPER_SERVO_OPENING_ANGLE)

        if self.state == "small_reverse":
            if timing(SMALL_REVERSE_TIME):
                self.vel(0.0, 0.0)
                self.state = "armup2"
                logging.info(self.state)
            else:
                self.vel(-0.1, 0.0)

        if self.state == "armup2":
            #logging.info(self.state)
            if timing(ARM_UP_TIME):
                self.state = "idle"
                logging.info(self.state)
                self.run_target0 = None
                self.run_target1 = None
                self.run_target = None
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)
        
        if self.state == "LED turn":
            #logging.info(self.state)
            _, trg =  target_nav(self.x + LED_R*np.cos(LED_TH + self.theta), 
                                    self.y + LED_R*np.sin(LED_TH + self.theta),
                                    self.theta,
                                    self.led_target[0],
                                    self.led_target[1])
            ang = (int(180*trg/PI) - LED_SERVO_OFFSET)%360
            
            if ang >175:
                self.vel(0.0, W_FINE)
            else:
                self.vel(0.0, 0.0)
                self.led(True)
                self.servo(MoveServo.SERVO_LED, ang)
                self.state = "idle"
                logging.info(self.state)
                self.led_target = []
        
        if self.state =="goto":
            move_lin, move_ang, ready = vel_control(Vec2(self.x, self.y),
                                                   self.theta, 
                                                   self.sensor_values,
                                                   robot_target=Vec2(self.run_target[0], self.run_target[1]))
            if ready:
                move_lin, move_ang = 0.0,0.0
                self.state = "idle"
                logging.info(self.state)
                self.run_target = None
            
            self.vel(move_lin, move_ang)

        if self.state =="pickup1":
            #logging.info(self.state)
            if timing(ARM_DOWN_TIME):
                self.state = "pickup2"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_DOWN_ANGLE)


        if self.state =="pickup2":
            #logging.info(self.state)
            if timing(GRASP_TIME):
                self.state = "pickup3"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_GRIPPER, GRIPPER_SERVO_CLOSING_ANGLE)

        if self.state =="pickup3":
            #logging.info(self.state)
            if timing(ARM_UP_TIME):
                self.state = "idle"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)

        if self.state == "putdown1":
            #logging.info(self.state)
            if timing(ARM_DOWN_TIME):
                self.state = "putdown2"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_DOWN_ANGLE)

        if self.state == "putdown2":
            #logging.info(self.state)
            if timing(RELEASE_TIME):
                self.state = "putdown3"
                logging.info(self.state)  
            else:
                self.servo(MoveServo.SERVO_GRIPPER, GRIPPER_SERVO_OPENING_ANGLE)

        if self.state == "putdown3":
            #logging.info(self.state)
            if timing(ARM_UP_TIME):
                self.state = "idle"
                logging.info(self.state)
            else:
                self.servo(MoveServo.SERVO_ARM, ARM_SERVO_UP_ANGLE)


        if self.state =="turnto":
            
            _, trg =  target_nav(self.x, 
                                     self.y,
                                     self.theta,
                                     self.turn_target[0],
                                     self.turn_target[1])

            if abs(trg)<3*eps_ANG:
                move_lin, move_ang = 0.0, 0.0
                self.state = "idle"
                logging.info(self.state)
                self.turn_target = []
            else:
                move_lin, move_ang = 0.0, np.sign(trg)*W_FINE
            
            self.vel(move_lin, move_ang)
        
        if self.state == "go str":
  
            if timing(abs(self.move_time)):
                move_lin, move_ang = 0.0,0.0
                self.state = "idle"
                logging.info(self.state)
                self.move_time = None
            else:
                move_lin, move_ang = np.sign(self.move_time)*FINE_MOVEMENT_SPEED,0.0
            
            self.vel(move_lin, move_ang)

        if self.state == "turn_rad":
  
            DELTATH = angle_calc(self.theta, self.angle_target)
            if abs(DELTATH)<3*eps_ANG:
                move_lin, move_ang = 0.0, 0.0
                self.state = "idle"
                logging.info(self.state)
                self.angle_target = None
            else:
                move_lin, move_ang = 0.0, np.sign(DELTATH)*W_FINE/2
            
            self.vel(move_lin, move_ang)



        



def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
          





        




