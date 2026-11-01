import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ObstacleCube(Node):
    def __init__(self):
        super().__init__('obstacle_cube')
        self.publisher = self.create_publisher(Twist, 'robot1/cube_cmd_robot1', 10)
        self.subscription = self.create_subscription(
            Odometry,
            '/robot1/cube_odom_robot1',
            self.update_position,
            10
        )
        self.timer = self.create_timer(0.1, self.move_cube)
        self.direction = 1
        self.move_command = Twist()
        self.current_y = None
        self.min_y = -78
        self.max_y = -62

    def update_position(self, odom: Odometry):
        self.current_y = odom.pose.pose.position.y
        if self.direction == 1 and self.current_y > self.max_y:
            self.direction = -1
        elif self.direction == -1 and self.current_y < self.min_y:
            self.direction = 1

    def move_cube(self):
        self.move_command.linear.y = 2.0 * self.direction
        self.publisher.publish(self.move_command)

def main(args=None):
    rclpy.init(args=args)
    obstacle_cube_node = ObstacleCube()
    rclpy.spin(obstacle_cube_node)
    obstacle_cube_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()