
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.move_turtle)
        self.get_logger().info('Turtle controller has started.')

    def move_turtle(self):
        msg = Twist()
        
        time.sleep(1)
        # Линейные скорости
        msg.linear.x = 1.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        

        # Угловые скорости
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
    
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent velocities - Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')





def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
