import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped, Polygon, Point32
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf2_ros
from tf2_geometry_msgs import PointStamped as TF2PointStamped
from tf2_geometry_msgs import do_transform_point
import math

class LaserScanNode(Node):
    def __init__(self):
        super().__init__('laser_scan_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_scan_callback,
            10)

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
            x_center = (leg_positions[0][0] + leg_positions[1][0]) / 2.0
            y_center = (leg_positions[0][1] + leg_positions[1][1]) / 2.0

            point_shelf_legs = PointStamped()
            point_shelf_legs.header.frame_id = msg.header.frame_id
            point_shelf_legs.header.stamp = msg.header.stamp
            point_shelf_legs.point.x = x_center
            point_shelf_legs.point.y = y_center
            point_shelf_legs.point.z = 0.0

            try:
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, msg.header.stamp)
                transformed_point = do_transform_point(point_shelf_legs, transform)

                self.x_transformed = transformed_point.point.x
                self.y_transformed = transformed_point.point.y
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Failed to transform point: {}'.format(str(e)))
                return False

    def laser_scan_callback(self, msg):
        self.detect_and_publish_shelf_legs(msg)
        print(self.x_transformed, self.y_transformed)

class NavigatorNode(BasicNavigator):
    def __init__(self):
        super().__init__()

    def set_initial_pose(self, initial_position):
        initial_pose_msg = PoseStamped()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.pose.position.x = initial_position['x']
        initial_pose_msg.pose.position.y = initial_position['y']
        initial_pose_msg.pose.orientation.z = initial_position['z']
        initial_pose_msg.pose.orientation.w = initial_position['w']
        
        self.setInitialPose(initial_pose_msg)

    def send_robot_to_goal(self, goal_position):
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = 'map'
        goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
        goal_pose_msg.pose.position.x = goal_position['x']
        goal_pose_msg.pose.position.y = goal_position['y']
        goal_pose_msg.pose.orientation.z = goal_position['z']
        goal_pose_msg.pose.orientation.w = goal_position['w']
        
        self.get_logger().info(f'Sending robot to ({goal_position["x"]}, {goal_position["y"]})')
        self.goToPose(goal_pose_msg)

class ShelfLiftController(Node):
    def __init__(self):
        super().__init__('shelf_lift_controller')

        self.elevator_up_publisher = self.create_publisher(Empty, '/elevator_up', 10)
        self.elevator_down_publisher = self.create_publisher(Empty, '/elevator_down', 10)

        self.global_footprint_publisher = self.create_publisher(Polygon, '/global_costmap/footprint', 10)
        self.local_footprint_publisher = self.create_publisher(Polygon, '/local_costmap/footprint', 10)

    def lift_shelf_up(self):

        up_msg = Empty()
        self.elevator_up_publisher.publish(up_msg)
        self.get_logger().info('Sent signal to lift shelf up.')
        self.change_footprint(0.3)

    def lower_shelf_down(self):

        down_msg = Empty()
        self.elevator_down_publisher.publish(down_msg)
        self.get_logger().info('Sent signal to lower shelf down.')
        self.change_footprint(0.15)

    def change_footprint(self, radius):
        # Create a circular footprint with the specified radius
        footprint_msg = Polygon()
        for angle in range(0, 360, 10):  # Change step for higher/lower resolution
            rad = angle * 3.14159265 / 180.0
            point = Point32()
            point.x = radius * math.cos(rad)
            point.y = radius * math.sin(rad)
            footprint_msg.points.append(point)
        
        # Publish the new footprint to both global and local costmaps
        self.global_footprint_publisher.publish(footprint_msg)
        self.local_footprint_publisher.publish(footprint_msg)
        self.get_logger().info(f'Changed robot footprint to radius: {radius} meters.')

def main(args=None):
    rclpy.init(args=args)

    navigator_node = NavigatorNode()
    laser_node = LaserScanNode()
    lift_controller = ShelfLiftController()

    increase_modulus_by = 0.45 #to move it right under shelf
    
    initial_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    preload_position = {'x': 4.7, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    after_load_position = {'x': 5.4, 'y': 0.0, 'z': 1.0, 'w': 0.0}
    preload_position_real = {'x': 4.7, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    after_load_position_real = {'x': 5.4, 'y': 0.0, 'z': 1.0, 'w': 0.0}
    unload_position = {'x': 2.53, 'y': 1.0, 'z': -0.7, 'w': 0.7}
    after_load_position_real = {'x': 5.4, 'y': 0.0, 'z': 1.0, 'w': 0.0}
    unload_position_real = {'x': 2.53, 'y': 1.0, 'z': -0.7, 'w': 0.7}
    after_unload_position = {'x': 2.53, 'y': 0.0, 'z': 1.0, 'w': 0.0}

    #navigator_node.set_initial_pose(initial_position)
    navigator_node.waitUntilNav2Active()
    navigator_node.send_robot_to_goal(preload_position)

    while not navigator_node.isTaskComplete():
        while laser_node.x_transformed == 0.0 and laser_node.y_transformed == 0.0:
            rclpy.spin_once(laser_node)

    midlegs_position = {'x': laser_node.x_transformed, 'y': laser_node.y_transformed, 'z': -0.7, 'w': 0.7}        
    under_shelf_position = {
        'x': laser_node.x_transformed,
        'y': (abs(laser_node.y_transformed) + increase_modulus_by) * (-1 if laser_node.y_transformed < 0 else 1),
        'z': 0.7,
        'w': 0.7
    } 

    result_preload = navigator_node.getResult()

    if result_preload == TaskResult.SUCCEEDED:
        navigator_node.send_robot_to_goal(midlegs_position)
        while not navigator_node.isTaskComplete():
            print("Moving to legs midpoint")
        result_midpoint = navigator_node.getResult()

        if result_midpoint == TaskResult.SUCCEEDED:
            navigator_node.send_robot_to_goal(under_shelf_position)
            while not navigator_node.isTaskComplete():
                print("Moving to under shelf position")
            under_shelf_result = navigator_node.getResult()

            if under_shelf_result == TaskResult.SUCCEEDED:
                lift_controller.lift_shelf_up()
                navigator_node.send_robot_to_goal(after_load_position)
                while not navigator_node.isTaskComplete():
                    print("Moving to after load position")
                after_load_result = navigator_node.getResult()

                if after_load_result == TaskResult.SUCCEEDED:
                    # Perform actions after reaching the after load position
                    print("Reached after load position successfully")
                    navigator_node.send_robot_to_goal(unload_position)
                    while not navigator_node.isTaskComplete():
                        print("Moving to unload position")
                    unload_result = navigator_node.getResult()

                    if unload_result == TaskResult.SUCCEEDED:
                        lift_controller.lower_shelf_down()
                        navigator_node.send_robot_to_goal(after_unload_position)
                        while not navigator_node.isTaskComplete():
                            print("Moving to unload position")
                        after_unload_result = navigator_node.getResult()
                        if after_unload_result == TaskResult.SUCCEEDED:
                            navigator_node.send_robot_to_goal(initial_position)
           
if __name__ == '__main__':
    main()