import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty, Bool, Int32, Int32MultiArray
from robot_interfaces.msg import Direction, MoveServo

import math
import serial

ser = serial.Serial("/dev/ttyACM0", 9600)


class UartProcessor(Node):

    def __init__(self):
        super().__init__('uart_node')
        self.localization_publisher = self.create_publisher(Direction, '/loc_direction', 10)
        self.gripper_sensor_publisher = self.create_publisher(Int32MultiArray, "/sensor_results", 10)
        
        self.servo_subscription = self.create_subscription(MoveServo, "/move_servo", self.move_servo, 10)
        self.led_subscription = self.create_subscription(Bool, "/led", self.set_led, 10)
        self.localization_subscription = self.create_subscription(Empty, "/localize", self.localize, 10)
        self.localization_subscription_2 = self.create_subscription(Bool, "/localize2", self.localize2, 10)
        self.localization_subscription_3 = self.create_subscription(Int32, "/localize3", self.localize3, 10)
        self.gripper_sensors_subscription = self.create_subscription(Empty, "/gripper_sensors", self.gripper_sensors, 10)
        self.uart_timer = self.create_timer(0.02, self.uart_read)
        self.input_buffer = ""
        self.get_logger().info(f'UART node initialized')

    def move_servo(self, msg_in: MoveServo):
        msg_out = f"SERVO {msg_in.id} {msg_in.angle}\n"
        ser.write(bytes(msg_out, 'ascii'))
        self.get_logger().info(f'Servo move published: {msg_out}')

    def set_led(self, msg_in: Bool):
        enabled = 1 if msg_in.data else 0
        msg_out = f"LED {enabled}\n"
        ser.write(bytes(msg_out, 'ascii'))

    def localize(self, _: Empty):
        ser.write(b"LOCALIZE\n")
        self.get_logger().info(f'LOCALIZE serial command sent')
        
    def localize2(self, msg_in: Bool):
        enabled = 1 if msg_in.data else 0
        msg_out = f"LOCALIZE2 {enabled}\n"
        ser.write(bytes(msg_out, 'ascii'))
        self.get_logger().info(f'{msg_out.strip()} serial command sent')

    def localize3(self, msg_in: Int32):
        enabled = msg_in.data
        msg_out = f"LOCALIZE3 {enabled}\n"
        ser.write(bytes(msg_out, 'ascii'))
        self.get_logger().info(f'{msg_out.strip()} serial command sent')
        
    def gripper_sensors(self, _: Empty):
        ser.write(b"SENSORS 1\n")
        self.get_logger().info(f'SENSORS command send')

    def uart_read(self):
        # self.get_logger().info(f'Serial bytes waiting: {ser.in_waiting}')
        if ser.in_waiting == 0:
            return
        
        # self.get_logger().info(f'Serial something received')
        msg_read = ser.read(ser.in_waiting)
        try:
            self.input_buffer += msg_read.decode("ascii")
        except UnicodeDecodeError:
            self.get_logger().error(f'Error decoding serial data: {msg_read}')
            return

        pos = self.input_buffer.find('\n')
        while pos != -1:
            # self.get_logger().info(f'Serial data with EOL received')
            #try:
            msg_str = self.input_buffer[:pos]
            self.input_buffer = self.input_buffer[pos + 1:]
            # addition for sensor readouts
            if msg_str.split(" ")[0] == "SENSORS":
                self.get_logger().info(f"sensor msg: {msg_str}")
                self.publish_sensor_results(msg_str)
            else:
                self.publish_direction(msg_str)
            pos = self.input_buffer.find('\n')
            #except Exception as e:
            #    self.get_logger().error(f"Exception e: {e}")
            #    self.get_logger().error(f"couldnt process uart msg: {self.input_buffer[:pos]}")
            #    #self.input_buffer = ""
                
    def publish_direction(self, raw_in):
        try:
            raw_dir, raw_angle = raw_in.split(' ')
        except:
            self.get_logger.error(f"could split values: {raw_in}")
        msg = Direction()
        msg.direction = {
            "NW": Direction.NORTH_WEST,
            "NE": Direction.NORTH_EAST,
            "SW": Direction.SOUTH_WEST,
            "SE": Direction.SOUTH_EAST
        }.get(raw_dir, Direction.NONE)

        try:
            msg.angle = float(raw_angle) / 180.0 * math.pi
        except ValueError:
            msg.angle = 0.0

        self.localization_publisher.publish(msg)
        self.get_logger().info(f'{raw_in} direction published')
        
    def publish_sensor_results(self, raw_in):
        try: 
            values = raw_in.split(" ")
        except:
            self.get_logger().error(f"couldnt split values {raw_in}")
        msg = Int32MultiArray()
        msg.data = list(map(int,values[1:]))
        self.gripper_sensor_publisher.publish(msg)
        
        

def main(args=None):
    rclpy.init(args=args)
    uart_processor = UartProcessor()
    try:
        rclpy.spin(uart_processor)
    except KeyboardInterrupt:
        pass    


if __name__ == '__main__':
    main()
