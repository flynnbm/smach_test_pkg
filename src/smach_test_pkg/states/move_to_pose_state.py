import smach
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from robot_common_manip.srv import MoveToPose

class MoveToPoseState(smach.State):
    def __init__(self, node, userdata, timeout_sec: float = 5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node
        self.userdata = userdata
        self.timeout_sec = timeout_sec
        self.client = node.create_client(MoveToPose, 'move_to_pose')

    def on_enter(self):
        self.node.get_logger().info("[MoveToPoseState] Entering state.")

    def execute(self, _):
        self.on_enter()

        if not self.userdata.grasp_poses:
            self.node.get_logger().error("[MoveToPoseState] No pose found in userdata.")
            return 'failure'

        target_pose = self.userdata.grasp_poses[-1]

        if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
            self.node.get_logger().error("[MoveToPoseState] Service not available.")
            return 'failure'

        request = MoveToPose.Request()
        request.target_pose = target_pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None and future.result().success:
            self.node.get_logger().info("[MoveToPoseState] Motion to pose succeeded.")
            return 'success'
        else:
            self.node.get_logger().error("[MoveToPoseState] Motion to pose failed.")
            return 'failure'

    def on_exit(self):
        self.node.get_logger().info("[MoveToPoseState] Exiting state.")