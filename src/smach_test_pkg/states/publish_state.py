import smach
from std_msgs.msg import String

class PublishState(smach.State):
    def __init__(self, node, name, message):
        smach.State.__init__(self, outcomes=['next'])
        self.node = node
        self.name = name
        self.message = message
        self.publisher = node.create_publisher(String, 'smach_test', 10)

    def on_enter(self, userdata):
        self.node.get_logger().info(f"[{self.name}] Entering state.")

    def execute(self, userdata):
        self.on_enter(userdata)

        try:
            msg = String()
            msg.data = f"[{self.name}] {self.message}"
            self.publisher.publish(msg)
            self.node.get_logger().info(f"Published: {msg.data}")
            return 'next'
        except Exception as e:
            self.node.get_logger().error(f"[{self.name}] Execution failed: {e}")
            return 'next'
        finally:
            self.on_exit(userdata)

    def on_exit(self, userdata):
        self.node.get_logger().info(f"[{self.name}] Exiting state.")