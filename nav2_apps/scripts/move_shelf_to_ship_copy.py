import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
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

            try:

                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())

                transformed_x = x_center + transform.transform.translation.x
                transformed_y = y_center + transform.transform.translation.y

                self.x_transformed = transformed_x
                self.y_transformed = -transformed_y
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error('Failed to transform point: {}'.format(str(e)))
                return False
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

def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorNode()
    laser_node = LaserScanNode()
    
    initial_position = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    preload_position = {'x': 4.7, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    

    navigator_node.send_robot_to_goal(preload_position)

    while not navigator_node.isTaskComplete():
        while laser_node.x_transformed == 0.0 and laser_node.y_transformed == 0.0:
            executor = MultiThreadedExecutor()
            executor.add_node(laser_node)
            executor.spin_once()

    midlegs_position = {'x': laser_node.x_transformed, 'y': -laser_node.y_transformed, 'z': -0.7, 'w': -0.7}        

    result_preload = navigator_node.getResult()

    if result_preload == TaskResult.SUCCEEDED:
        navigator_node.send_robot_to_goal(midlegs_position)


if __name__ == '__main__':
    main()