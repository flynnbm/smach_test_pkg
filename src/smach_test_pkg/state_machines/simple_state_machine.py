import smach
from smach_test_pkg.states.publish_state import PublishState

# Main state machine
def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = rclpy.create_node('smach_test_node')

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('STATE_ONE', PublishState(node, 'STATE_ONE', 'Starting up...'),
                               transitions={'next': 'STATE_TWO'})
        smach.StateMachine.add('STATE_TWO', PublishState(node, 'STATE_TWO', 'Doing stuff...'),
                               transitions={'next': 'STATE_THREE'})
        smach.StateMachine.add('STATE_THREE', PublishState(node, 'STATE_THREE', 'Wrapping up!'),
                               transitions={'next': 'done'})

    sm.execute()
    node.get_logger().info("State machine finished.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()