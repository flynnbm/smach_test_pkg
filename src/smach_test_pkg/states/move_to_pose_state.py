import smach
from geometry_msgs.msg import Pose
from robot_common_manip.srv import MoveToPose

class MoveToPoseState(smach.State):
    def __init__(self, node, target_pose: Pose, timeout_sec: float = 5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node
        self.target_pose = target_pose
        self.timeout_sec = timeout_sec
        self.client = node.create_client(MoveToPose, 'move_to_pose')

    def on_enter(self, userdata):
        self.node.get_logger().info("[MoveToPoseState] Entering state.")

    def execute(self, userdata):
        self.on_enter(userdata)

        if not self.client.wait_for_service(timeout_sec=self.timeout_sec):
            self.node.get_logger().error("[MoveToPoseState] Service not available.")
            return 'failure'

        request = MoveToPose.Request()
        request.target_pose = self.target_pose

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None and future.result().success:
            self.node.get_logger().info("[MoveToPoseState] Motion to pose succeeded.")
            return 'success'
        else:
            self.node.get_logger().error("[MoveToPoseState] Motion to pose failed.")
            return 'failure'

    def on_exit(self, userdata):
        self.node.get_logger().info("[MoveToPoseState] Exiting state.")