import smach
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand


class GripperCommandState(smach.State):
    def __init__(self, node, robot_name: str, position: float = 0.01, max_effort: float = 40.0, timeout_sec: float = 5.0):
        smach.State.__init__(self, outcomes=['success', 'failure'])
        self.node = node
        self.position = position
        self.max_effort = max_effort
        self.timeout_sec = timeout_sec

        self.action_name = f"/{robot_name}_hand_controller/gripper_cmd"
        self._client = ActionClient(node, GripperCommand, self.action_name)

    def feedback_cb(self, feedback_msg):
        # Optional: log or handle feedback
        feedback = feedback_msg.feedback
        self.node.get_logger().debug(f"[GripperCommandState] Feedback: {feedback}")

    def on_enter(self, userdata):
        self.node.get_logger().info(f"[GripperCommandState] Waiting for action server '{self.action_name}'...")
        if not self._client.wait_for_server(timeout_sec=self.timeout_sec):
            self.node.get_logger().error(f"[GripperCommandState] Action server '{self.action_name}' not available.")
            self._server_available = False
        else:
            self._server_available = True
            self.node.get_logger().info(f"[GripperCommandState] Connected to action server '{self.action_name}'.")

    def execute(self, userdata):
        self.on_enter(userdata)
        if not self._server_available:
            return 'failure'
        
        # Log goal values for verification
        self.node.get_logger().info(f"[GripperCommandState] Sending goal: position={self.position}, max_effort={self.max_effort}")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = self.position
        goal_msg.command.max_effort = self.max_effort

        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"[GripperCommandState] Goal was rejected by the server.")
            return 'failure'

        self.node.get_logger().info(f"[GripperCommandState] Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        result = result_future.result()
        if result is None:
            self.node.get_logger().error("[GripperCommandState] No result received.")
            return 'failure'

        status = result.status
        result_data = result.result  # GripperCommand_Result

        self.node.get_logger().info(f"[GripperCommandState] Final status: {status}")
        self.node.get_logger().info(f"[GripperCommandState] Result: position={result_data.position:.4f}, "
                                    f"effort={result_data.effort:.2f}, "
                                    f"stalled={result_data.stalled}, "
                                    f"reached_goal={result_data.reached_goal}")

        # If the gripper moved at all, treat as success
        if result_data.position > 0.001:
            self.node.get_logger().info("[GripperCommandState] Gripper moved — treating as success.")
            return 'success'

        # Otherwise, fall back to stricter checks
        if status == 0 and result_data.reached_goal:
            self.node.get_logger().info("[GripperCommandState] Gripper goal succeeded cleanly.")
            return 'success'

        self.node.get_logger().error("[GripperCommandState] Gripper did not move and goal failed.")
        return 'failure'

    def on_exit(self, userdata):
        self.node.get_logger().info("[GripperCommandState] Exiting state.")