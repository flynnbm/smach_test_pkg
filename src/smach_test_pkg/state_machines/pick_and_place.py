import smach
from geometry_msgs.msg import Pose
from smach_test_pkg.states.take_snapshot_state import TakeSnapshotState
from smach_test_pkg.states.move_to_named_pose_state import MoveToNamedPoseState
from smach_test_pkg.states.move_to_pose_state import MoveToPoseState
from smach_test_pkg.states.gripper_command_state import GripperCommandState
from smach_test_pkg.states.grasp_state import GraspState

# Shared Data
class UserData():
    def __init__(self):
        self.grasp_poses = []
        self.pointclouds = []
        self.robot_name = ""
        self.ready_pose = ""
        self.dropoff_pose = ""
        self.inspect_pose = ""
        self.max_open_val = 0.0
        self.max_close_val = 0.0

# Main state machine
def main():
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = rclpy.create_node('smach_test_node')

    userdata = UserData()

    sm = smach.StateMachine(outcomes=['done'])

    node.declare_parameter("robot_name", "panda")
    node.declare_parameter("ready_pose", "ready")
    node.declare_parameter("dropoff_pose", "dropoff")
    node.declare_parameter("inspect_pose", "inspect")
    node.declare_parameter("max_open_val", 0.03)
    node.declare_parameter("max_close_val", 0.008)

    # Retrieve parameter values
    userdata.robot_name = node.get_parameter("robot_name").get_parameter_value().string_value
    userdata.ready_pose = node.get_parameter("ready_pose").get_parameter_value().string_value
    userdata.dropoff_pose = node.get_parameter("dropoff_pose").get_parameter_value().string_value
    userdata.inspect_pose = node.get_parameter("inspect_pose").get_parameter_value().string_value
    userdata.max_open_val = node.get_parameter("max_open_val").get_parameter_value().double_value
    userdata.max_close_val = node.get_parameter("max_close_val").get_parameter_value().double_value

    with sm:
        smach.StateMachine.add('MOVE_TO_START', MoveToNamedPoseState(node, userdata.ready_pose),
                           transitions={'success': 'MOVE_TO_INSPECT',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_INSPECT', MoveToNamedPoseState(node, userdata.inspect_pose, 5.0),
                           transitions={'success': 'TAKE_SNAPSHOT',
                                        'failure': 'MOVE_TO_INSPECT'})        
        smach.StateMachine.add('TAKE_SNAPSHOT', TakeSnapshotState(node, userdata),
                               transitions={'success': 'MOVE_TO_NEUTRAL',
                                            'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_NEUTRAL', MoveToNamedPoseState(node, userdata.ready_pose, 5.0),
                           transitions={'success': 'CALCULATE_GRASP_POSE',
                                        'failure': 'done'})
        # Opportunity to filter image further if required
        smach.StateMachine.add('CALCULATE_GRASP_POSE', GraspState(node, userdata, 5.0),
                               transitions={'success': 'MOVE_TO_APPROACH_POSE',
                                            'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_APPROACH_POSE', MoveToPoseState(node, userdata, 5.0),
                           transitions={'success': 'OPEN_GRIPPER',
                                        'failure': 'done'})    
        smach.StateMachine.add('OPEN_GRIPPER', GripperCommandState(node, userdata),
                           transitions={'success': 'MOVE_TO_GRASP_POSE',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_GRASP_POSE', MoveToPoseState(node, userdata, 5.0),
                           transitions={'success': 'CLOSE_GRIPPER',
                                        'failure': 'done'}) 
        smach.StateMachine.add('CLOSE_GRIPPER', GripperCommandState(node, userdata),
                           transitions={'success': 'MOVE_TO_RETREAT_POSE',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_RETREAT_POSE', MoveToPoseState(node, userdata, 5.0),
                           transitions={'success': 'MOVE_TO_NEUTRAL_2',
                                        'failure': 'done'}) 
        smach.StateMachine.add('MOVE_TO_NEUTRAL_2', MoveToNamedPoseState(node, userdata.ready_pose, 5.0),
                           transitions={'success': 'MOVE_TO_DROPOFF',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_DROPOFF', MoveToNamedPoseState(node, userdata.dropoff_pose, 5.0),
                           transitions={'success': 'OPEN_GRIPPER_2',
                                        'failure': 'done'})
        smach.StateMachine.add('OPEN_GRIPPER_2', GripperCommandState(node, userdata),
                           transitions={'success': 'MOVE_TO_END',
                                        'failure': 'done'})
        smach.StateMachine.add('MOVE_TO_END', MoveToNamedPoseState(node, userdata.ready_pose, 5.0),
                           transitions={'success': 'done',
                                        'failure': 'done'})

    sm.execute()
    node.get_logger().info("State machine finished.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

