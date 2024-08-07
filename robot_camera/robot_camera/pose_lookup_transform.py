import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
#from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

class FrameListener(Node):

    def __init__(self):
        super().__init__('pose_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create lookup transform pose publisher
        self.pose_publisher = self.create_publisher(PoseStamped, 'tag38/pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
        # Store the static tag frame to map transform
        try:
            self.tag38_to_map = self.tf_buffer.lookup_transform(
                'tag_frame_38',
                'map',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform tag_frame_38 to map')
            self.tag38_to_map = TransformStamped()

    def on_timer(self):
        # Look up for the transformation between the measured tag frame and robot
        try:
            measured_tag38 = self.tf_buffer.lookup_transform(
                'tag36h11:38',
                'base_footprint',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform tag36h11:38 to base_footprint')
            return

        # find the unknown map to base_footprint transform
        robot_tag38 = measured_tag38 * self.tag38_to_map

        robot_pose_msg = PoseStamped()
        robot_pose_msg.header.stamp = self.get_clock().now().to_msg()
        robot_pose_msg.header.frame_id = "map"
        robot_pose_msg.pose.position.x = robot_tag38.transform.translation.x
        robot_pose_msg.pose.position.y = robot_tag38.transform.translation.y
        robot_pose_msg.pose.position.z = robot_tag38.transform.translation.z
        robot_pose_msg.pose.orientation = robot_tag38.transform.rotation
        self.pose_publisher.publish(robot_pose_msg)

def main():
    rclpy.init()
    pose_transform_node = FrameListener()
    
    try:
        rclpy.spin(pose_transform_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
