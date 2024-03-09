import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, TransformStamped, PointStamped
from std_msgs.msg import Header
from rclpy.executors import MultiThreadedExecutor

import math
import tf2_ros
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import time
from copy import deepcopy

class WareHouseManager(Node):
    def __init__(self):
        super().__init__('WareHouseManager') 
        self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, 10)

        self.intensity_threshold = 6000.0
        self.x_transformed = 0.0
        self.y_transformed = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def detect_and_publish_shelf_legs(self, msg):
        leg_positions = []
        is_currently_high_intensity = False
        current_angle = msg.angle_min

        for i, intensity in enumerate(msg.intensities):
            if intensity >= self.intensity_threshold:
                if not is_currently_high_intensity:
                    x = msg.ranges[i] * math.cos(current_angle)
                    y = msg.ranges[i] * math.sin(current_angle)
                    leg_positions.append((x, y))
                    is_currently_high_intensity = True
            else:
                is_currently_high_intensity = False
            current_angle += msg.angle_increment

        if len(leg_positions) == 2:

            print("Legs founded!")

            x_center = (leg_positions[0][0] + leg_positions[1][0]) / 2.0
            y_center = (leg_positions[0][1] + leg_positions[1][1]) / 2.0

            try:

                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id,  msg.header.stamp)
                trans = transform.transform.translation

                self.x_transformed = x_center + trans.x
                self.y_transformed = y_center + trans.y

                print(x_center, y_center)

                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Error transform: %s' % str(e))
                return False

        return False

    def laser_scan_callback(self, msg):
        self.detect_and_publish_shelf_legs(msg)
        print(self.x_transformed, self.y_transformed)


def main():

    rclpy.init()
    check_positions = {
        "shelf_zone_point":[4.69, 0.0], 
        "under_shelf_point":[0.0, 0.0],
        "shipping_point": [2.59, 0.96],
        "start_position": [0.0, 0.0]
    }

    shelf_zone_location_point = 'shelf_zone_point'
    under_shelf_point = 'under_shelf_point'
    #shipping_destination_point = 'shipping_point'
    #start_position_point = 'start_position'

    warehouse_manager = WareHouseManager()
    navigator = BasicNavigator()

    executor = MultiThreadedExecutor()
    executor.add_node(warehouse_manager)
    executor.spin()


    # Set demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate fully
    navigator.waitUntilNav2Active()

    # Set shelf_zone
    shelf_item_pose = PoseStamped()
    shelf_item_pose.header.frame_id = 'map'
    shelf_item_pose.header.stamp = navigator.get_clock().now().to_msg()
    shelf_item_pose.pose.position.x = check_positions[shelf_zone_location_point][0]
    shelf_item_pose.pose.position.y = check_positions[shelf_zone_location_point][1]
    shelf_item_pose.pose.orientation.z = 0.0
    shelf_item_pose.pose.orientation.w = 1.0
    print('Received request for item picking at ' + shelf_zone_location_point + '.')
    navigator.goToPose(shelf_item_pose)

    while not navigator.isTaskComplete():
        check_positions["under_shelf_point"] = [warehouse_manager.x_transformed, warehouse_manager.y_transformed]

    result = navigator.getResult()

    print(result)
    if result == TaskResult.SUCCEEDED:

        print('Got product from ' + under_shelf_point +
            '! Bringing product to shipping destination (' + under_shelf_point + ')...')
        under_shelf_destination = PoseStamped()
        under_shelf_destination.header.frame_id = 'map'
        under_shelf_destination.header.stamp = navigator.get_clock().now().to_msg()
        under_shelf_destination.pose.position.x = check_positions[under_shelf_point][0]
        under_shelf_destination.pose.position.y = check_positions[under_shelf_point][1]
        under_shelf_destination.pose.orientation.z = -0.7
        under_shelf_destination.pose.orientation.w = -0.7
        navigator.goToPose(under_shelf_destination)

    elif result == TaskResult.CANCELED:
        print('Task at ' + shelf_zone_location_point +
            ' was canceled. Returning to staging point...')
        initial_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(initial_pose)

    elif result == TaskResult.FAILED:
        print('Task at ' + shelf_zone_location_point + ' failed!')
        exit(-1)

    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()