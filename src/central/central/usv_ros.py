import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
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

    def publish(self):
        left_thrust_msg = Float64()
        left_thrust_msg.data = self.left_thrust
        self.left_thrust_publisher.publish(left_thrust_msg)

        right_thrust_msg = Float64()
        right_thrust_msg.data = self.right_thrust
        self.right_thrust_publisher.publish(right_thrust_msg)


def main():
    rclpy.init()
    usv_ros = USV_ROS()
    rclpy.spin(usv_ros)
    usv_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
