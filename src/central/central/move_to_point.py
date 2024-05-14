import math

import rclpy
from rclpy.node import Node
from usv_ros import USV_ROS


def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


class PointController(Node):
    def __init__(
        self,
        usv,
        destination_point,
        base_speed=10,  # Base speed of the boat
        kp=10,  # Proportional gain for steering
    ):
        super().__init__("point_controller")
        self.usv: USV_ROS = usv
        self.destination_point = destination_point
        self.base_speed = base_speed
        self.kp = kp

    def steer_to_destination(self):
        if self.destination_point is None or self.usv.pose is None:
            return

        x_current, y_current = (
            self.usv.pose.position.x,
            self.usv.pose.position.y,
        )
        x_dest, y_dest = self.destination_point

        angle_to_dest = math.atan2(y_dest - y_current, x_dest - x_current)

        orientation = self.usv.pose.orientation
        _, _, yaw = quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        steering_angle = angle_to_dest - yaw

        self.usv.left_thrust = self.base_speed - (self.kp * steering_angle)
        self.usv.right_thrust = self.base_speed + (self.kp * steering_angle)

        self.usv.publish()

        print(f"Orientation: {math.degrees(yaw):.2f} degrees")
        print(f"Angle to destination: {math.degrees(angle_to_dest):.2f} degrees")
        print(f"Steering angle: {math.degrees(steering_angle):.2f} degrees")


def main():
    rclpy.init()
    usv_ros = USV_ROS()
    destination_point = (-528, 190)
    controller = PointController(usv_ros, destination_point)

    while rclpy.ok():
        rclpy.spin_once(usv_ros)
        controller.steer_to_destination()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
