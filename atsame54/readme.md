### `uart_node`

Responsible for translating the communication between ROS2 and the SAM E54. Every message goes through this node.

Topics this node is listening to:
* `/move_servo` with message type `robot_interfaces/msg/MoveServo`
* `/led` of type `std_msgs/msg/Bool`
* `/localize` of type `std_msgs/msg/Empty`

Corresponding commands sent over UART:
* `SERVO {servo id} {target angle}`
* `LED {1/0 signalizing on/off}`
* `LOCALIZE` (without any additional parameters)
All messages must end with `\n`.

Topics this node publishes to:
* `/loc_direction` of type `robot_interfaces/msg/Direction`

Corresponding commands received over UART:
* `NW`, `NE`, `SW` or `SE` — all of them closed with a `\n`. If there was no response from the corners, the message received is `NO`.
