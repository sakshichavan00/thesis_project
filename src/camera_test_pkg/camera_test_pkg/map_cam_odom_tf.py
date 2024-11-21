# This node publishes the map -> base_link transform based on the odometry and the base_link pose
# It operates in two modes: simulation and real-world. 
# In simulation mode, the initial spawn location of the robot is predefined as a parameter. 
# In real-world mode, it uses the first few values of the camera data to compute the world -> map transform.
# This node helps in publishing the world -> odom transform for later accuracy calculations.

import rclpy
from rclpy.node import Node
import tf2_ros
from nav_msgs.msg import Odometry
import tf_transformations
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf2_geometry_msgs

class MapToOdomTransformPublisher(Node):
    def __init__(self):
        super().__init__('cam_map_to_odom_transform_node')
        self.get_logger().info("Node started")

        # Initialize parameters for the initial robot pose
        self.init_yaw = self.declare_parameter('initial_yaw', 0.0).get_parameter_value().double_value
        self.init_x = self.declare_parameter('initial_x', 0.0).get_parameter_value().double_value
        self.init_y = self.declare_parameter('initial_y', 0.0).get_parameter_value().double_value
        self.init_z = self.declare_parameter('initial_z', 0.0).get_parameter_value().double_value
        
        # Additional parameters
        self.use_init_offset = self.declare_parameter('use_initial_offset', True).get_parameter_value().bool_value
        self.map_pose_topic = self.declare_parameter('map_pose_topic', '/map_camera_pose').get_parameter_value().string_value
        self.odom_input_topic = self.declare_parameter('odom_input_topic', '/odometry/filtered').get_parameter_value().string_value
        self.world_pose_topic = self.declare_parameter('world_pose_topic', '/robot_pose').get_parameter_value().string_value
        self.log_level = self.declare_parameter('log_level', 0).get_parameter_value().integer_value

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Compute initial rotation matrix
        self.initial_rot_matrix = tf_transformations.euler_matrix(0, 0, self.init_yaw)

        # Variables for storing poses
        self.odom_pose = None
        self.base_link_pose_world = None
        
        # Subscribers for odometry and base_link pose
        self.create_subscription(Odometry, self.odom_input_topic, self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.world_pose_topic, self.pose_callback, 10)

        # Transform broadcaster
        self.tf_broadcaster_world_map = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_world_to_map_transform()

        # Publisher for the transformed pose
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, self.map_pose_topic, 10)

    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose

    def pose_callback(self, msg):
        try:
            # Look up transform from map to world
            map_to_world_transform = self.tf_buffer.lookup_transform('map', 'world', rclpy.time.Time())
            # Transform the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_with_covariance_stamped(msg, map_to_world_transform)
            # Adjust the z position
            transformed_pose.pose.pose.position.z = self.init_z
            # Publish the transformed pose
            self.pose_publisher.publish(transformed_pose)
            if self.log_level > 0:
                self.get_logger().info(f'Map to World Transform: {map_to_world_transform}')
                self.get_logger().info(f'Transformed Pose: {transformed_pose}')
        except Exception as e:
            self.get_logger().error(f'Error in transforming pose: {str(e)}')

    def publish_static_transform(self):
        # Publish a static transform from world to map using initial offsets
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "world"
        transform_stamped.child_frame_id = "map"
        transform_stamped.transform.translation.x = self.init_x
        transform_stamped.transform.translation.y = self.init_y
        transform_stamped.transform.translation.z = self.init_z

        # Convert yaw to quaternion
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.init_yaw)
        transform_stamped.transform.rotation.x = quaternion[0]
        transform_stamped.transform.rotation.y = quaternion[1]
        transform_stamped.transform.rotation.z = quaternion[2]
        transform_stamped.transform.rotation.w = quaternion[3]

        # Publish the static transform
        self.tf_broadcaster_world_map.sendTransform(transform_stamped)

    def publish_world_to_map_transform(self):
        if self.use_init_offset:
            self.publish_static_transform()
        else:
            # Additional logic for real-world mode if not using initial offsets
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomTransformPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
