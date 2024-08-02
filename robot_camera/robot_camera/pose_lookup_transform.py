import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
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

    def on_timer(self):
        # Look up for the transformation between target_frame and source_frame
        # and send pose for target tag
        try:
            t = self.tf_buffer.lookup_transform(
                'tag36h11:38',
                'tag_frame_38',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform tag_frame_38 to tag36h11:38')
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = t.transform.translation.x
        msg.pose.position.y = t.transform.translation.y
        msg.pose.position.z = t.transform.translation.z
        msg.pose.orientation.w = t.transform.rotation.w
        msg.pose.orientation.x = t.transform.rotation.x
        msg.pose.orientation.y = t.transform.rotation.y
        msg.pose.orientation.z = t.transform.rotation.z
        self.pose_publisher.publish(msg)

def main():
    rclpy.init()
    pose_transform_node = FrameListener()
    
    try:
        rclpy.spin(pose_transform_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
