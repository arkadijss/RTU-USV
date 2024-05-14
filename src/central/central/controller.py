import rclpy
from pynput import keyboard
from usv_ros import USV_ROS


class Controller:
    def __init__(self, usv_ros):
        self.usv_ros: USV_ROS = usv_ros
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.key_pressed = None

    def on_press(self, key):
        try:
            self.key_pressed = key.char
        except AttributeError:
            # Ignore non-printable keys.
            pass

    def start(self):
        self.listener.start()
        while rclpy.ok():
            if self.key_pressed == "w":
                self.usv_ros.accelerate_forward(10)
            elif self.key_pressed == "s":
                self.usv_ros.accelerate_backward(10)
            elif self.key_pressed == "a":
                self.usv_ros.steer_left(10)
            elif self.key_pressed == "d":
                self.usv_ros.steer_right(10)
            elif self.key_pressed == "q":
                break  # Quit if 'q' is pressed
            self.key_pressed = None
            self.usv_ros.publish()

    def stop(self):
        self.listener.stop()


def main():
    rclpy.init()
    usv_ros = USV_ROS()
    controller = Controller(usv_ros)
    controller.start()
    controller.stop()
    usv_ros.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
