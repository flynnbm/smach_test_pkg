import smach
import rclpy
from geometry_msgs.msg import Pose
from robot_common_manip.srv import FakeGrasp

class TakeSnapshotState(smach.State):
    def __init__(self, node, userdata):
        super().__init__(outcomes=['success', 'failure'])
        self.node = node
        self.userdata = userdata
        self.client = node.create_client(FakeGrasp, 'grasp_pose')

    def on_enter(self):
        self.node.get_logger().info("[TakeSnapshotState] Entering state.")

    def execute(self, _):
        self.node.get_logger().info("[TakeSnapshotState] Waiting for service...")

        # if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
        #     self.node.get_logger().error("[TakeSnapshotState] Service not available.")
        #     return 'failure'

        # request = FakeGrasp.Request()
        # future = self.client.call_async(request)
        # rclpy.spin_until_future_complete(self.node, future)

        # if future.result() is not None:
        #     # grasp_pose = future.result().grasp_pose
        #     # self.userdata.grasp_poses.append(grasp_pose)
        #     # update with actual snapshot taken
        #     self.node.get_logger().info(f"[TakeSnapshotState] Got grasp pose and stored in userdata.")
        #     return 'success'
        # else:
        #     self.node.get_logger().error("[TakeSnapshotState] Failed to call service.")
        #     return 'failure'

        return 'success'
        
    def on_exit(self):
        self.node.get_logger().info("[TakeSnapshotState] Exiting state.")