import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node
from pynput import keyboard

from usv import USV


class USV_ROS(USV, Node):
    def __init__(
        self,
        left_thrust_topic="/wamv/thrusters/left/thrust",
        right_thrust_topic="/wamv/thrusters/right/thrust",
    ):
        super().__init__()
        Node.__init__(self, "usv_ros")
        self.left_thrust_publisher = self.create_publisher(
            Float64, left_thrust_topic, 10
        )
        self.right_thrust_publisher = self.create_publisher(
            Float64, right_thrust_topic, 10
        )
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.key_pressed = None

        self.listener.start()
        self.start()

    def on_press(self, key):
        try:
            self.key_pressed = key.char
        except AttributeError:
            # Ignore non-printable keys.
            pass

    def publish(self):
        left_thrust_msg = Float64()
        left_thrust_msg.data = self.left_thrust
        self.left_thrust_publisher.publish(left_thrust_msg)

        right_thrust_msg = Float64()
        right_thrust_msg.data = self.right_thrust
        self.right_thrust_publisher.publish(right_thrust_msg)

    def start(self):
        while rclpy.ok():
            if self.key_pressed == "w":
                self.accelerate_forward(10)
            elif self.key_pressed == "s":
                self.accelerate_backward(10)
            elif self.key_pressed == "a":
                self.steer_left(10)
            elif self.key_pressed == "d":
                self.steer_right(10)
            self.key_pressed = None

            self.publish()


def main():
    rclpy.init()
    usv_ros = USV_ROS()
    rclpy.spin(usv_ros)
    usv_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
