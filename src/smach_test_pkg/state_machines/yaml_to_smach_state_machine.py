import os
import yaml
import rclpy
from rclpy.node import Node
import smach

from ament_index_python.packages import get_package_share_directory

from smach_test_pkg.states.publish_state import PublishState

def load_state_config():
    pkg_path = get_package_share_directory('smach_test_pkg')
    config_path = os.path.join(pkg_path, 'config', 'publish_states.yaml')
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def main():
    rclpy.init()
    node = rclpy.create_node('smach_yaml_loader_node')

    config = load_state_config()
    state_defs = config.get('states', [])

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        for i, state_def in enumerate(state_defs):
            name = state_def['name']
            message = state_def.get('message', '')
            next_state = state_defs[i + 1]['name'] if i + 1 < len(state_defs) else 'done'

            smach.StateMachine.add(
                name,
                PublishState(node, name, message),
                transitions={'next': next_state}
            )

    outcome = sm.execute()
    node.get_logger().info(f"State machine completed with outcome: {outcome}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()