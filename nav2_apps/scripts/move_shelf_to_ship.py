import rclpy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

def autolocalize():
    rclpy.init()
    node = rclpy.create_node('autolocalize')

    # Perform auto localization
    client = node.create_client(Empty, '/reinitialize_global_localization')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Service /reinitialize_global_localization not available, waiting...')
    request = Empty.Request()
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Auto localization completed successfully!')
    else:
        node.get_logger().info('Failed to perform auto localization')

    # Control the robot to perform rotations for better localization
    publisher = node.create_publisher(Twist, '/robot/cmd_vel', 10)
    cmd = Twist()

    def rotate_left(timer):
        node.get_logger().info('Rotating left for 5 seconds...')
        cmd.angular.z = 0.5  # Example angular speed
        publisher.publish(cmd)
        timer.cancel()

    def rotate_right(timer):
        node.get_logger().info('Rotating right for 10 seconds...')
        cmd.angular.z = -0.5  # Example angular speed
        publisher.publish(cmd)
        timer.cancel()

    def stop_rotation(timer):
        node.get_logger().info('Stopping rotation...')
        cmd.angular.z = 0
        publisher.publish(cmd)

    # Rotate left for 5 seconds
    rotate_left_timer = node.create_timer(5, lambda: rotate_left(rotate_left_timer))

    # Rotate right for 10 seconds
    rotate_right_timer = node.create_timer(10, lambda: rotate_right(rotate_right_timer))

    # Rotate left for 5 seconds
    rotate_left_timer_2 = node.create_timer(5, lambda: rotate_left(rotate_left_timer_2))

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    autolocalize()
