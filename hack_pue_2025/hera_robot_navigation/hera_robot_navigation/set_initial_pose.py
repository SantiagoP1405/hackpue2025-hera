import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import numpy as np

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Esperar a que Nav2 esté activo
    navigator.waitUntilNav2Active()

    # Pose inicial (ajusta si quieres)
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(0.0))
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    navigator.setInitialPose(pose)
    navigator.get_logger().info("Posición inicial establecida.")

    rclpy.shutdown()

if _name_ == '_main_':
    main()