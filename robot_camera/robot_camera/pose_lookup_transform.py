import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf
import numpy as np

class FrameListener(Node):

    def __init__(self):
        super().__init__('pose_transformer')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create lookup transform pose publisher
        self.pose33_publisher = self.create_publisher(PoseStamped, 'tag33/pose', 1)
        self.pose34_publisher = self.create_publisher(PoseStamped, 'tag34/pose', 1)
        self.pose35_publisher = self.create_publisher(PoseStamped, 'tag35/pose', 1)
        self.pose36_publisher = self.create_publisher(PoseStamped, 'tag36/pose', 1)
        self.pose37_publisher = self.create_publisher(PoseStamped, 'tag37/pose', 1)
        self.pose38_publisher = self.create_publisher(PoseStamped, 'tag38/pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)
        
        self.got_static_transform = [False, False, False, False, False, False]
        self.got_static_transforms = False
        
        self.tag_transform_matrix = np.zeros((6,4,4))

    def on_timer(self):
        # First get the static transform, then continue with measured transform
        # Could not get the static transform from inside the constructor, don't know why????
        if self.got_static_transforms:
            for measured_tag in range(33, 39):
                try:
                    tag_frame_name = 'tag36h11:' + str(measured_tag)
                    tag_to_base = self.tf_buffer.lookup_transform(
                        tag_frame_name,
                        'base_footprint',
                        rclpy.time.Time())
                    v = tag_to_base.transform.translation
                    q = tag_to_base.transform.rotation
                    matrix2 = np.dot(tf.translation_matrix([v.x, v.y, v.z]), tf.quaternion_matrix([q.x, q.y, q.z, q.w]))
                    matrix3 = np.dot(self.tag_transform_matrix[measured_tag - 33], matrix2)
                    v = tf.translation_from_matrix(matrix3)
                    q = tf.quaternion_from_matrix(matrix3)

                    robot_pose_msg = PoseStamped()
                    robot_pose_msg.header.stamp = tag_to_base.header.stamp
                    robot_pose_msg.header.frame_id = "map"
                    robot_pose_msg.pose.position.x = v[0]
                    robot_pose_msg.pose.position.y = v[1]
                    robot_pose_msg.pose.position.z = v[2]
                    robot_pose_msg.pose.orientation.w = q[3]
                    robot_pose_msg.pose.orientation.x = q[0]
                    robot_pose_msg.pose.orientation.y = q[1]
                    robot_pose_msg.pose.orientation.z = q[2]
                    
                    if measured_tag == 33:
                        self.pose33_publisher.publish(robot_pose_msg)
                    elif measured_tag == 34:
                        self.pose34_publisher.publish(robot_pose_msg)
                    elif measured_tag == 35:
                        self.pose35_publisher.publish(robot_pose_msg)
                    elif measured_tag == 36:
                        self.pose36_publisher.publish(robot_pose_msg)
                    elif measured_tag == 37:
                        self.pose37_publisher.publish(robot_pose_msg)
                    elif measured_tag == 38:
                        self.pose38_publisher.publish(robot_pose_msg)
                except TransformException as ex:
                    pass
        else:
            # Look up the static transforms first
            for static_tag in range(33, 39):
                if not self.got_static_transform[static_tag - 33]:
                    try:
                        tag_frame_name = 'tag_frame_' + str(static_tag)
                        map_to_tag = self.tf_buffer.lookup_transform(
                            'map',
                            tag_frame_name,
                            rclpy.time.Time())
                        v = map_to_tag.transform.translation
                        q = map_to_tag.transform.rotation
                        self.tag_transform_matrix[static_tag - 33] = np.dot(
                            tf.translation_matrix([v.x, v.y, v.z]), 
                            tf.quaternion_matrix([q.x, q.y, q.z, q.w]))
                        self.got_static_transform[static_tag - 33] = True
                    except TransformException as ex:
                        self.get_logger().info('Could not transform tag_frame_' + str(static_tag) + ' to map')
            self.got_static_transforms = all(self.got_static_transform)
       
def main():
    rclpy.init()
    pose_transform_node = FrameListener()
    
    try:
        rclpy.spin(pose_transform_node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
