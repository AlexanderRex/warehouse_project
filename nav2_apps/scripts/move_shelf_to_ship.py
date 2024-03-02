import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import PointStamped

from visualization_msgs.msg import Marker
from tf2_geometry_msgs import PointStamped
import math
import tf2_ros
from rclpy.duration import Duration



class WareHouseManager(Node):
    def __init__(self):
        super().__init__('autolocalize')
        self.publisher = self.create_publisher(Twist, '/robot/cmd_vel', 10)
        self.cli = self.create_client(Empty, '/reinitialize_global_localization')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/reinitialize_global_localization service not available, waiting again...')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)
        self.rotation_sequence = [(0.5, 10), (-0.5, 15), (0.5, 5)]
        self.rotation_timer = None
        self.rotation_index = 0

        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.intensity_threshold = 6000.0  
        self.x_center = 0.0
        self.y_center = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def reinitialize_global_localization(self):
        self.get_logger().info('Reinitializing global localization...')
        req = Empty.Request()
        self.cli.call_async(req)

    def publish_marker_at_shelf_legs(self, x, y, stamp):
        try:
            point_lidar_frame = PointStamped()
            point_lidar_frame.header.frame_id = "robot_front_laser_base_link"
            point_lidar_frame.header.stamp = stamp
            point_lidar_frame.point.x = x
            point_lidar_frame.point.y = y
            point_lidar_frame.point.z = 0.0

            # Преобразуем точку в фрейм карты, если трансформация доступна
            if self.tf_buffer.can_transform('map', point_lidar_frame.header.frame_id, stamp):
                point_map_frame = self.tf_buffer.transform(point_lidar_frame, "map", timeout=Duration(seconds=1.0))

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = stamp
                marker.ns = "shelf_legs"
                marker.id = 0
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point_map_frame.point.x
                marker.pose.position.y = point_map_frame.point.y
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                self.marker_publisher.publish(marker)
        except tf2_ros.LookupException as e:
            self.get_logger().error(f'No transform available: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f'Error in transform: {e}')

    def detect_and_publish_shelf_legs(self):
        leg_count = 0
        leg_positions = []
        is_currently_high_intensity = False
        current_angle = self.current_scan.angle_min

        for i, intensity in enumerate(self.current_scan.intensities):
            if intensity >= self.intensity_threshold:
                if not is_currently_high_intensity:
                    x = self.current_scan.ranges[i] * math.cos(current_angle)
                    y = self.current_scan.ranges[i] * math.sin(current_angle)
                    leg_positions.append((x, y))
                    leg_count += 1
                    is_currently_high_intensity = True
            else:
                is_currently_high_intensity = False
            current_angle += self.current_scan.angle_increment

        if leg_count == 2:
            self.x_center = (leg_positions[0][0] + leg_positions[1][0]) / 2.0
            self.y_center = (leg_positions[0][1] + leg_positions[1][1]) / 2.0

        legs_detected = (leg_count == 2)
        return legs_detected

    def laser_scan_callback(self, msg):
        self.current_scan = msg
        if self.detect_and_publish_shelf_legs():
            self.publish_marker_at_shelf_legs(self.x_center, self.y_center, msg.header.stamp)

    def rotate_robot(self):
        if self.rotation_index < len(self.rotation_sequence):
            angular_speed, duration = self.rotation_sequence[self.rotation_index]
            twist = Twist()
            twist.angular.z = angular_speed
            self.publisher.publish(twist)
            self.rotation_timer = self.create_timer(duration, self.handle_rotation_end)
        else:
            self.get_logger().info('Rotation sequence completed.')
            self.rotation_index = 0  # Reset the index if you plan to call rotate_robot again in the future.

    def handle_rotation_end(self):
        self.rotation_timer.cancel()  # It's important to cancel the timer to avoid multiple callbacks.
        if self.rotation_index < len(self.rotation_sequence) - 1:
            self.rotation_index += 1
            self.rotate_robot()
        else:
            self.publisher.publish(Twist())  # Stop the robot at the end of the sequence.
            self.get_logger().info('Final rotation completed. Stopping the robot.')

if __name__ == '__main__':
    rclpy.init()
    warehouse_manager = WareHouseManager()
    current_time = warehouse_manager.get_clock().now()

    try:
        warehouse_manager.reinitialize_global_localization()
        warehouse_manager.rotate_robot()
        rclpy.spin(warehouse_manager)
    except KeyboardInterrupt:
        pass
    finally:
        warehouse_manager.destroy_node()
        rclpy.shutdown()
