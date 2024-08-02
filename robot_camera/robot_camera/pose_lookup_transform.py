import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import Pose

class FrameListener(Node):

    def __init__(self):
        super().__init__('pose_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create lookup transform pose publisher
        self.pose_publisher = self.create_publisher(Pose, 'tag38/pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Look up for the transformation between target_frame and source_frame
        # and send pose for target tag
        try:
            t = self.tf_buffer.lookup_transform(
                'tag_frame_38',
                'apriltag nodes name for tag 38',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info('Could not transform to_frame_rel to from_frame_rel')
            return

        msg = Pose()
        msg.Point.x = t.transform.translation.x
        msg.Point.y = t.transform.translation.y
        msg.Point.z = t.transform.translation.z
        msg.Quaternion.w = t.transform.rotation.w
        msg.Quaternion.x = t.transform.rotation.x
        msg.Quaternion.y = t.transform.rotation.y
        msg.Quaternion.z = t.transform.rotation.z
        self.publisher.publish(msg)

def main():
    rclpy.init()
    pose_transform_node = FrameListener()
    
    try:
        rclpy.spin(pose_transform_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
