#!/usr/bin/env python3
from .FuncAndConst import *

class UserCommandNode (Node):

    def __init__(self):
        super().__init__(node_name="user_command_node")
        self.cmd_vel_publisher_ = self.create_publisher(
        Command, "/command", 10)
        self.get_logger().info("Command manager has been started.")
        self.cmd=Command()
        
        while True:
        
            optext = input("Please give command:")
            
            if optext == "LED ON":
                xled = float(input("Please give the x coordinate of the point to enlight:"))
                yled = float(input("Please give the y coordinate of the point to enlight:"))
                self.cmd.values = [xled,yled]
                self.cmd.opcode = Command.LED_ON
            elif optext == "RESCUE":
                xpick = float(input("Please give the x coordinate of the object location:"))
                ypick = float(input("Please give the y coordinate of the object location:"))
                xleave = float(input("Please give the x coordinate of the point to leave the object:"))
                yleave = float(input("Please give the y coordinate of the point to leave the object:"))
                self.cmd.values = [xpick,ypick,xleave,yleave]
                self.cmd.opcode = Command.MAIN_OPERATION
            elif optext == "LED OFF":
                self.cmd.values = []
                self.cmd.opcode = Command.LED_OFF
            elif optext == "CALIBRATE":  
                self.cmd.values = []
                self.cmd.opcode = Command.CALIBRATE
            elif optext == "GOTO":
                xgo = float(input("Please give the x coordinate of the point to go:"))
                ygo = float(input("Please give the y coordinate of the point to go:"))
                self.cmd.values = [xgo,ygo]
                self.cmd.opcode = Command.GOTO
            elif optext == "PICKUP":
                self.cmd.opcode = Command.PICKUP
                self.cmd.values = []
            elif optext == "PUTDOWN":
                self.cmd.opcode = Command.PUTDOWN
                self.cmd.values = []
            elif optext == "STATUS":
                self.cmd.opcode = Command.GIVESTATE
                self.cmd.values = []
            elif optext == "TURNTO":
                self.cmd.opcode = Command.TURNTO
                xturn = float(input("Please give the x coordinate of the point to turn:"))
                yturn = float(input("Please give the y coordinate of the point to turn:"))
                self.cmd.values = [xturn, yturn]
            elif optext == "GO STRAIGHT":
                self.cmd.opcode = Command.GOSTR
                l=float(input("Please give the amount you want to go:"))
                self.cmd.values = [l]
            elif optext == "TURN":
                self.cmd.opcode = Command.TURN
                th = float(input("Please give the amount you want to turn:"))
                self.cmd.values = [th]
            elif optext == "PICKFROM":
                xpick = float(input("Please give the x coordinate of the object location:"))
                ypick = float(input("Please give the y coordinate of the object location:"))
                self.cmd.opcode = Command.PICKFROM
                self.cmd.values = [xpick, ypick]
            elif optext == "LEAVETO":
                xleave = float(input("Please give the x coordinate of the point to leave the object:"))
                yleave = float(input("Please give the y coordinate of the point to leave the object:"))
                self.cmd.values = [xleave,yleave]
                self.cmd.opcode = Command.LEAVETO
            else:
                print("Invalid Input")
                continue
            self.cmd_vel_publisher_.publish (self.cmd)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = UserCommandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass