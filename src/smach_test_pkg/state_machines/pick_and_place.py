import smach
from geometry_msgs.msg import Pose
from smach_test_pkg.states.publish_state import PublishState
from smach_test_pkg.states.move_to_named_pose_state import MoveToNamedPoseState
from smach_test_pkg.states.move_to_pose_state import MoveToPoseState
from smach_test_pkg.states.gripper_command_state import GripperCommandState
from smach_test_pkg.states.fake_grasp_state import FakeGraspState

# Shared Data
class UserData:
    def __init__(self):
        self.grasp_poses = []
        self.pointclouds = []

# Main state machine
def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = rclpy.create_node('smach_test_node')

    userdata = UserData()

    sm = smach.StateMachine(outcomes=['done'])

    with sm:
        smach.StateMachine.add('MOVE_TO_START', MoveToNamedPoseState(node, 'ready'),
                           transitions={'success': 'MOVE_TO_INSPECT',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_INSPECT', MoveToNamedPoseState(node, 'inspect'),
                           transitions={'success': 'TAKE_SNAPSHOT',
                                        'failure': 'done'})        
        #  placeholder state for getting image snapshot
        smach.StateMachine.add('TAKE_SNAPSHOT', PublishState(node, 'TAKE_SNAPSHOT', 'taking a snapshot...'),
                               transitions={'next': 'MOVE_TO_NEUTRAL'})
        smach.StateMachine.add('MOVE_TO_NEUTRAL', MoveToNamedPoseState(node, 'ready'),
                           transitions={'success': 'FILTER_IMAGE',
                                        'failure': 'done'})
        # placeholder state for image filtering -- pass in userdata for pointcloud
        smach.StateMachine.add('FILTER_IMAGE', PublishState(node, 'FILTER_IMAGE', 'filtering image...'),
                               transitions={'next': 'CALCULATE_GRASP_POSE'})
        # placeholder state for grasp pose calculation -- pass in userdata; pointclouds used, poses filled
        smach.StateMachine.add('CALCULATE_GRASP_POSE', FakeGraspState(node, userdata, 5.0),
                               transitions={'success': 'MOVE_TO_APPROACH_POSE',
                                            'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_APPROACH_POSE', MoveToPoseState(node, userdata, 5.0),
                           transitions={'success': 'OPEN_GRIPPER',
                                        'failure': 'done'})    
        smach.StateMachine.add('OPEN_GRIPPER', GripperCommandState(node, 'panda', 0.03, 40.0, 5.0),
                           transitions={'success': 'MOVE_TO_GRASP_POSE',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', PublishState(node, 'MOVE_TO_GRASP_POSE', 'moving to grasp pose...'),
                               transitions={'next': 'CLOSE_GRIPPER'})
        smach.StateMachine.add('CLOSE_GRIPPER', GripperCommandState(node, 'panda', 0.008, 40.0, 5.0),
                           transitions={'success': 'MOVE_TO_RETREAT_POSE',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_RETREAT_POSE', PublishState(node, 'MOVE_TO_RETREAT_POSE', 'moving to retreat pose...'),
                               transitions={'next': 'MOVE_TO_NEUTRAL_2'})
        smach.StateMachine.add('MOVE_TO_NEUTRAL_2', MoveToNamedPoseState(node, 'ready'),
                           transitions={'success': 'MOVE_TO_DROPOFF',
                                        'failure': 'done'})
        # temporarily using inspect pose to just move to a different position
        smach.StateMachine.add('MOVE_TO_DROPOFF', MoveToNamedPoseState(node, 'inspect'),
                           transitions={'success': 'OPEN_GRIPPER_2',
                                        'failure': 'done'})
        smach.StateMachine.add('OPEN_GRIPPER_2', GripperCommandState(node, 'panda', 0.03, 40.0, 5.0),
                           transitions={'success': 'MOVE_TO_END',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_END', MoveToNamedPoseState(node, 'ready'),
                           transitions={'success': 'done',
                                        'failure': 'done'})

    sm.execute()
    node.get_logger().info("State machine finished.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()