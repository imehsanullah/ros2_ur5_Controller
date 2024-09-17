
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rclpy.action import ActionClient
import threading

from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

class URController(Node):

    def __init__(self):
        super().__init__('ur_controller')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS profile, depth of the queue
        )
        self._goal_handle = None  # To store the current goal handle
        
        self.robot_joint_position= [0,1.57,-1.57,-1.57,1.57,-1.57] # [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]
        
  
    def send_goal(self, joint_positions):
        goal_msg = FollowJointTrajectory.Goal()

        # Define the joint names
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Create the trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = 5  # Move to position in 5 seconds

        # Create the trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = joint_names
        trajectory.points = [point]

        # Set the trajectory in the goal message
        goal_msg.trajectory = trajectory

        # Wait for the action server to be ready
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            return False

        # Send the goal and wait for it to be accepted
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False

        self.get_logger().info('Goal accepted')

        # Wait for the goal to complete
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Goal succeeded')
            return True
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code}')
            return False

   


    def bring_robot_to_init(self, joint_angles):
        JOINT_NAMES = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        # Cancel previous goal if any
        if self._goal_handle:
            self._goal_handle.cancel_goal_async()

        # Define single trajectory point
        position_target_point = JointTrajectoryPoint()
        position_target_point.positions = joint_angles
        position_target_point.time_from_start.sec = 3  # 1 second to complete the move
        position_target_point.time_from_start.nanosec = int(1e9 * 0.25)  # 0.25 seconds more (total 1.25 sec)

        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()

        # Set trajectory header and points
        goal_msg.trajectory.joint_names = JOINT_NAMES
        goal_msg.trajectory.points.append(position_target_point)
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()

        # Wait for the action server to be ready
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available")
            return False

        # Send the goal and wait for the result
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return False

        self.get_logger().info('Goal accepted')

        # Wait for the goal to complete
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('Goal succeeded')
            return True
        else:
            self.get_logger().error(f'Goal failed with error code: {result.error_code}')
            return False

            
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received')
        rclpy.shutdown()
        
    
    def joint_state_callback(self, data: JointState):
        """
        This function will be called whenever a new joint state message is received.
        :param data: The JointState message containing the current joint positions, velocities, and efforts.
        """
        self.robot_joint_position = data.position  # Store joint positions
        # self.get_logger().info(f'Updated joint positions: {self.robot_joint_position}')
    
    def get_robot_joint_angles(self):
        '''
        WARNING!! 
        1. JointState from real robot has different order. So rearranging the joint states in below order as per simualation (Position only). 
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ]   
        
        2. The Actual Robot joint possition is as followed:
            -0 shoulder_lift_joint 
            -1 elbow_joint 
            -2 wrist_2_joint
            -3 wrist_3_joint
            -4 shoulder_pan_joint
            -5 wrist_1_joint
            -6 robotiq_85_left_knuckle_joint
        
        3. The "shoulder_pan_joint" is also 1.57 radians off as compared to sim, so subtract that value as well.
        '''
        robot_joint_position = [self.robot_joint_position[4] - 1.57 ,self.robot_joint_position[0],self.robot_joint_position[1], 
                                self.robot_joint_position[5],self.robot_joint_position[2],self.robot_joint_position[3]]
        
        return robot_joint_position





class GripperActionClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')
        self._action_client = ActionClient(self, GripperCommand, '/robotiq_gripper_controller/gripper_cmd')

    def send_goal(self, position, max_effort):
        self.get_logger().info(f"Sending goal to gripper: position={position}, max_effort={max_effort}")

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        # Wait until the action server is ready
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server not available after waiting")
            return

        self.get_logger().info("Action server available, sending goal.")
        future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received feedback: {feedback_msg}")
        
    



class ROS2bridge:

    def __init__(self):
        rclpy.init()
        
        # Initialize both clients
        self.joint_trajectory_client = URController()
        self.gripper_client = GripperActionClient()
        
        # Subscribe to the /joint_states topic to get the robot's joint positions
        self.robot_joint_position = []
       

        # Use MultiThreadedExecutor to handle both nodes
        self.executor = MultiThreadedExecutor()

        # Add nodes to the executor
        self.executor.add_node(self.joint_trajectory_client)
        self.executor.add_node(self.gripper_client)

        # Start the executor in a separate thread
        self.executor_thread = threading.Thread(target=self.spin_executor, daemon=True)
        self.executor_thread.start()

    def spin_executor(self):
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()

    def send_joint_goal(self, joint_positions):
        self.joint_trajectory_client.send_goal(joint_positions)

    def send_gripper_goal(self, position, max_effort=50.0):
        self.gripper_client.send_goal(position, max_effort)

    def stop(self):
        if rclpy.ok():
            rclpy.shutdown()
        if self.executor_thread.is_alive():
            self.executor_thread.join()

    def get_robot_joint_angles(self):
        return self.joint_trajectory_client.get_robot_joint_angles()
    
    def bring_robot_to_init(self, joint_angles):
        self.joint_trajectory_client.bring_robot_to_init(joint_angles)

# You can use this class in another Python script without needing to call spin() manually
if __name__ == '__main__':
    bridge = ROS2bridge()

    # Example joint positions for the robot arm [1.57, -1.57, 1.57, -1.57, -1.57, -1.57]    [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    joint_positions = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
    bridge.send_joint_goal(joint_positions)

    # bridge.bring_robot_to_init(joint_positions)
    # Example command to close and open the gripper
    bridge.send_gripper_goal(0.0, 50.0)  # Close gripper
    # bridge.send_gripper_goal(0.0, 50.0)  # Open gripper

    print('Robot joint angles',bridge.get_robot_joint_angles())
    try:
        # Do other things while the executor runs in the background
        pass
    finally:
        # When you're done with the bridge
        bridge.stop()
