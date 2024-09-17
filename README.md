# ros2_example_pac

	
Код управления черепахой:

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
        
        # Линейные скорости
        msg.linear.x = 2.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        
        # Угловые скорости
        msg.angular.x = 0.5
        msg.angular.y = 0.0
        msg.angular.z = 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent velocities - Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')

def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



	
	Разбор кода
    1. Инициализация ROS 2 ноды:

       super().__init__('turtle_controller')
       Нода называется turtle_controller. Каждая нода в ROS является самостоятельным процессом и может взаимодействовать с другими через топики, сервисы и действия.
    2. Публикация сообщений:
       
       self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
       Нода создаёт паблишер (издатель) для топика /turtle1/cmd_vel, который публикует сообщения типа Twist. Этот тип сообщений содержит линейные и угловые скорости. Размер очереди сообщений равен 10, это значит, что нода может хранить до 10 сообщений до их обработки.
    3. Таймер и периодическая публикация:
       
       self.timer = self.create_timer(0.5, self.move_turtle)
       Таймер вызывает функцию move_turtle каждые 0.5 секунд. В ROS 2 таймеры полезны для выполнения задач с заданной периодичностью, например, для отправки команд движения.
    4. Отправка сообщения о движении:
       
       def move_turtle(self):
               msg = Twist()
              
             # Линейные скорости
               msg.linear.x = 2.0
               msg.linear.y = 0.0
               msg.linear.z = 0.0
               
             # Угловые скорости
               msg.angular.x = 0.5
               msg.angular.y = 0.0
           msg.angular.z = 1.0
           self.publisher_.publish(msg)
       В этой функции создаётся сообщение типа Twist, где линейные скорости задаются по осям x, y и z, а угловые — по x, y и z. В этом примере линейная скорость задаётся вдоль оси x, а угловая — вокруг оси z. Сообщение публикуется в топик, что управляет черепахой в симуляторе turtlesim.
    5. Логирование:
       
       self.get_logger().info(f'Sent velocities - Linear: ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}), Angular: ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})')
       Логгирование позволяет отслеживать, какие команды отправляются в топик.
    6. Инициализация ROS:
       
       rclpy.init()
       rclpy.spin(node)
       rclpy.shutdown()
       rclpy.init() запускает систему ROS 2, rclpy.spin(node) — это бесконечный цикл, который поддерживает работу ноды, пока она не будет остановлена. rclpy.shutdown() завершает работу ROS 2.
