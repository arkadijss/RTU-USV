class USV:
    def __init__(self):
        self.left_thrust = 0.0
        self.right_thrust = 0.0
        self.pose = None

    def accelerate_forward(self, speed):
        self.left_thrust += speed
        self.right_thrust += speed

    def accelerate_backward(self, speed):
        self.left_thrust -= speed
        self.right_thrust -= speed

    def steer_left(self, speed):
        self.left_thrust -= speed
        self.right_thrust += speed

    def steer_right(self, speed):
        self.left_thrust += speed
        self.right_thrust -= speed
