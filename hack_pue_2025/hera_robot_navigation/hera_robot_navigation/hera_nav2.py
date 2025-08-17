import rclpy # type: ignore
from rclpy.node import Node  # type: ignore
from nav2_simple_commander.robot_navigator import BasicNavigator # type: ignore
from geometry_msgs.msg import PoseStamped, PointStamped # type: ignore
import tf_transformations # type: ignore
import numpy as np # type: ignore

class PublishPoint(Node):
    def __init__(self):
        super().__init__('publish_point_node')
        self.nav = BasicNavigator()
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point', 
            self.objective_point_callback,
            10)
        initial_pose = create_pose_stamped(self.nav, 0., 0., 0.)
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active()
        
        self.get_logger().info('Posición inicial establecida. Establezca posición objetivo.')  # Publica cada segundo

    def objective_point_callback(self, msg: PointStamped):

        goal_pose = create_pose_stamped(self.nav, msg.point.x, msg.point.y, 0.0)
        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            self.nav.get_logger().info(f"x: {feedback.current_pose.pose.position.x}, y: {feedback.current_pose.pose.position.y}, Orientation: {feedback.current_pose.pose.orientation.z}, Distance remaining: {feedback.distance_remaining:.2f} m")
        result = self.nav.getResult()
        self.nav.get_logger().info(f"Navigation result: {result}")
    

def create_pose_stamped(navigator:BasicNavigator, x:float, y:float, q_z:float) -> PoseStamped:
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, np.deg2rad(q_z)) # type: ignore
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w

    return pose

def main():
    # --- Init
    rclpy.init()
    node = PublishPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

