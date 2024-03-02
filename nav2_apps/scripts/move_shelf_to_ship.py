import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class WareHouseManager(Node):
    def __init__(self):
        super().__init__('autolocalize')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        # Ожидаем, что сервис станет доступен
        self.cli = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/reinitialize_global_localization service not available, waiting again...')

    def reinitialize_global_localization(self):
        self.get_logger().info('Reinitializing global localization...')
        req = Empty.Request()
        self.cli.call_async(req)

    def rotate_robot(self, angular_speed, duration):
        self.get_logger().info(f'Rotating robot with angular speed: {angular_speed} rad/s for {duration} seconds')
        
        twist = Twist()
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
        
        self.create_timer(duration, lambda: self.publisher.publish(Twist()))

if __name__ == '__main__':
    rclpy.init()
    warehouse_manager = WareHouseManager()

    try:
        warehouse_manager.reinitialize_global_localization()
        rclpy.spin_once(warehouse_manager, timeout_sec=1)  # Даем время на обработку запроса сервиса
        
        # Повороты как указано
        warehouse_manager.rotate_robot(0.5, 10)  # Влево на 5 секунд
        rclpy.spin_once(warehouse_manager, timeout_sec=5)
        warehouse_manager.rotate_robot(-0.5, 15)  # Вправо на 10 секунд
        rclpy.spin_once(warehouse_manager, timeout_sec=10)
        warehouse_manager.rotate_robot(0.5, 10)  # Влево на 5 секунд
        rclpy.spin_once(warehouse_manager, timeout_sec=5)
        
    except KeyboardInterrupt:
        pass
    finally:
        warehouse_manager.destroy_node()
        rclpy.shutdown()
