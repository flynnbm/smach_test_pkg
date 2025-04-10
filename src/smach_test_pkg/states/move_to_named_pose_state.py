import smach
import rclpy
from rclpy.node import Node
from robot_common_manip.srv import MoveToNamedPose

class MoveToNamedPoseState(smach.State):
    def __init__(self, node: Node, target_pose_name: str, timeout_sec: float = 5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node
        self.target_pose_name = target_pose_name
        self.timeout_sec = timeout_sec
        self.client = node.create_client(MoveToNamedPose, 'move_to_named_pose')

    def on_enter(self, userdata):
        self.node.get_logger().info(f"[MoveToNamedPoseState] Entering state. Target: {self.target_pose_name}")

    def execute(self, userdata):
        self.on_enter(userdata)

        if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
            self.node.get_logger().error("[MoveToNamedPoseState] Service not available.")
            return 'failure'

        request = MoveToNamedPose.Request()
        request.target_name = self.target_pose_name

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None and future.result().success:
            self.node.get_logger().info(f"[MoveToNamedPoseState] Successfully moved to '{self.target_pose_name}'.")
            return 'success'
        else:
            self.node.get_logger().error(f"[MoveToNamedPoseState] Failed to move to '{self.target_pose_name}'.")
            return 'failure'

    def on_exit(self, userdata):
        self.node.get_logger().info("[MoveToNamedPoseState] Exiting state.")