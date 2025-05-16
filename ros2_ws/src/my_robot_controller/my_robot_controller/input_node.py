from .FuncAndConst import *

class InputProcessor(Node):

    def __init__(self):
        super().__init__('input_node')
        self.input_publisher = self.create_publisher(FinalInput, '/fine_input', 10)

        self.localization_subscription = self.create_subscription(Direction, "/loc_direction", self.localize, 10)
        self.sensor_subscription = self.create_subscription(Int32MultiArray, "/sensor_results", self.sensor, 10)
        self.command_subscription = self.create_subscription(Command, "/command", self.command_cb, 10)
        self.offset_subscription = self.create_subscription(Float32MultiArray, "/offset", self.offset_cb, 10)

        self.odom_sub = Subscriber(self, Odometry, "/odom", qos_profile=qos_profile_sensor_data)
        self.ir_sub = Subscriber(self, IrIntensityVector, "/ir_intensity", qos_profile=qos_profile_sensor_data)

        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.ir_sub],queue_size,coupling_delay)
        self.ts.registerCallback(self.callback)

        self.offset = [0,0,0]
        self.odom_calib = [0, 0, 0]
        self.odom_current = [0, 0, 0]
        self.command_flag = False
        self.command_values = []
        self.command_type = 0
        self.calib_message = 0
        self.fine_cmd = FinalInput()
        self.get_logger().info(f'Input merger node initialized')
        self.init_loc = [0, 0, 0]
        self.conv_angle=0
        self.calib_dirs=[]
        self.calib_angles = []
        self.gripper_full = False

 

    def localize(self, dir: Direction):
        self.calib_dirs.append(dir.direction)
        self.calib_angles.append(dir.angle)

    def command_cb(self, command:Command):
        self.command_flag = True
        self.command_type = command.opcode
        self.command_values = command.values

    def offset_cb(self, offset:Float32MultiArray):
        self.offset = offset.data.tolist()
        self.odom_calib = self.odom_current
        self.conv_angle = (self.offset[2] - self.odom_calib[2])%(2*PI)
    
    def sensor(self, sensor_data:Int32MultiArray):
        self.gripper_full = (sum(sensor_data.data.tolist())>0)
        #X=sensor_data.data.tolist()
        #self.gripper_full = X[1]>0
            

    def callback(self, odom: Odometry, ir: IrIntensityVector):
        sensor_array=[0]*7
        # sensor_array=[-1]*7        dist = -1 / 20 * math.log((value + 1) / 3800), log(0) at first, math error
        self.odom_current = odom_reader(odom)  #odom value to be processed
        for reading in ir.readings:
            sensor_array[IR_SENSORS[reading.header.frame_id]] = reading.value

        
        fine_loc = process_loc(self.odom_current, self.offset, self.odom_calib, self.conv_angle)


        self.fine_cmd.sensor_values = sensor_array
        self.fine_cmd.x = fine_loc[0]
        self.fine_cmd.y = fine_loc[1]
        self.fine_cmd.theta = fine_loc[2]
        self.fine_cmd.command_type = self.command_type
        self.fine_cmd.command_values = self.command_values
        self.fine_cmd.command_flag = self.command_flag
        self.fine_cmd.calib_dir = self.calib_dirs 
        self.fine_cmd.calib_angle = self.calib_angles
        self.fine_cmd.gripper_full = self.gripper_full
                        
        
        self.input_publisher.publish(self.fine_cmd)
        self.command_flag = False
        self.command_values = []
        self.command_type = 0
        self.calib_dirs=[]
        self.calib_angles = []
        self.gripper_full = False

   
def main(args=None):
    rclpy.init(args=args)
    uart_processor = InputProcessor()
    try:
        rclpy.spin(uart_processor)
    except KeyboardInterrupt:
        pass    


