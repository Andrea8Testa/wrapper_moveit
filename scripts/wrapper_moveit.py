#!/usr/bin/env python3.8

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from control_msgs.msg import GripperCommandAction, GripperCommandGoal, GripperCommandFeedback, GripperCommandResult
from trajectory_msgs.msg import JointTrajectory
from hrii_gri_interface.srv import SetGripperData
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np


class BridgeNode:
    def __init__(self):
        rospy.init_node('wrapper_moveit')
        namespace = "wrapper_moveit"
        robot_name = rospy.get_param('~robot_id', 0)
        print("robot_id: ", robot_name)
        if not type(robot_name) == str:
            raise Exception("Param 'arm_id' not found")
        action_topic_name = namespace + '/follow_joint_trajectory'
        gripper_topic_name = namespace + '_gripper/gripper_action'
        target_joint_topic_name = rospy.get_param('~target_joint_topic_name', 0)
        target_array_topic_name = rospy.get_param('~target_array_topic_name', 0)
        target_gripper_topic_name = rospy.get_param('~target_gripper_topic_name', 0)
        joint_state_topic_name = robot_name + '/joint_states'
        gripper_state_topic_name = robot_name + '/gripper/width'
        # Set up action server
        self.action_server = actionlib.SimpleActionServer(action_topic_name, FollowJointTrajectoryAction, self.execute_trajectory, False)
        self.action_server.start()

        self.gripper_action_server = actionlib.SimpleActionServer(gripper_topic_name, GripperCommandAction, self.gripper_command, False)
        self.gripper_action_server.start()

        # Initialize communication with your controller here
        self.target_joint_pub = rospy.Publisher(namespace + target_joint_topic_name, JointState, queue_size=1)
        self.target_array_pub = rospy.Publisher(robot_name + target_array_topic_name, Float64MultiArray, queue_size=1)
        self.target_gripper_pub = rospy.Publisher(robot_name + target_gripper_topic_name, Float64, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(joint_state_topic_name, JointState, self.joint_feedback)
        self.gripper_state_sub = rospy.Subscriber(gripper_state_topic_name, Float64, self.gripper_feedback)

        # Initialize messages
        self.target_joint = JointState()
        self.target_array = Float64MultiArray()
        self.target_gripper = Float64()

        rospy.spin()

    def execute_trajectory(self, goal):
        print("Executing the trajectory planned")
        # Extract trajectory from the goal
        trajectory = goal.trajectory
        num_points = len(trajectory.points)
        # Convert trajectory to commands for your controller here
        for i in range(num_points-1):
            # Sleep until it's time for the next point in the trajectory
            duration = (trajectory.points[i + 1].time_from_start - trajectory.points[i].time_from_start).to_sec()
            rate = rospy.Rate(100)  # Set the desired rate of 100 Hz
            init_time = (rospy.Time.now()).to_sec()
            end_time = (rospy.Time.now().to_sec() + duration)

            # Monitor the execution status and provide feedback to MoveIt
            while rospy.Time.now().to_sec() < end_time:
                # Extract the target joint positions from the current point
                new_target_positions = np.array(trajectory.points[i+1].positions)
                new_target_velocities = np.array(trajectory.points[i+1].velocities)
                new_target_effort = np.array(trajectory.points[i+1].effort)
                old_target_positions = np.array(trajectory.points[i].positions)
                old_target_velocities = np.array(trajectory.points[i].velocities)
                old_target_effort = np.array(trajectory.points[i].effort)
                target_positions = old_target_positions*(end_time - rospy.Time.now().to_sec())/duration +\
                                   new_target_positions*(rospy.Time.now().to_sec() - init_time)/duration
                target_velocities = old_target_velocities*(end_time - rospy.Time.now().to_sec())/duration +\
                                    new_target_velocities*(rospy.Time.now().to_sec() - init_time)/duration
                target_effort = old_target_effort*(end_time - rospy.Time.now().to_sec())/duration +\
                                new_target_effort*(rospy.Time.now().to_sec() - init_time)/duration

                # Create a sensor_msgs/JointState message
                self.target_joint = JointState()
                self.target_joint.header = trajectory.header
                self.target_joint.name = trajectory.joint_names
                self.target_joint.position = target_positions
                self.target_joint.velocity = target_velocities
                self.target_joint.effort = target_effort

                # Publish the sensor_msgs/JointState message
                self.target_joint_pub.publish(self.target_joint)

                # Create a std_msgs/Float64MultiArray message
                self.target_array = Float64MultiArray()
                for j in range(0, len(self.target_joint.position)):
                    self.target_array.data.append(self.target_joint.position[j])

                # Publish the std_msgs/Float64MultiArray message
                self.target_array_pub.publish(self.target_array)

                # Publish feedback at 100 Hz
                feedback = FollowJointTrajectoryFeedback()
                # Update the feedback message based on the execution status
                feedback.joint_names = self.target_joint.name
                feedback.desired.positions = self.target_joint.position
                feedback.desired.velocities = self.target_joint.velocity
                feedback.desired.effort = self.target_joint.effort

                feedback.actual.positions = self.joint_position_feedback
                feedback.actual.velocities = self.joint_velocity_feedback
                feedback.actual.effort = self.joint_effort_feedback

                for j in range(0, len(self.target_joint.position)):
                    feedback.error.positions.append(self.target_joint.position[j] - self.joint_position_feedback[j])
                    feedback.error.velocities.append(self.target_joint.velocity[j] - self.joint_velocity_feedback[j])

                # Publish feedback to MoveIt
                self.action_server.publish_feedback(feedback)

                rate.sleep()

        result = FollowJointTrajectoryResult()

        # Update the result messages based on the execution status
        result.error_code = FollowJointTrajectoryResult.SUCCESSFUL

        # Handle the result based on the execution status
        if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(result)

        print("Done")

    def gripper_command(self, goal):
        print("Setting the gripper command")
        position_command = goal.command.position
        max_effort_command = goal.command.max_effort

        # Not working on the real robot
        # Create a std_msgs/Float64 message
        #self.target_gripper = Float64()
        #self.target_gripper.data = position_command
        # Publish the std_msgs/Float64 message
        #self.target_gripper_pub.publish(self.target_gripper)

        if position_command < 0.01:
            velocity = 1.0   # Set the desired velocity
            force = 20       # Set the desired force
            print("grasping...")
            success = self.call_grasp_from_outside(velocity, force)
            if success:
                print("grasping completed")
            else:
                print("error occurred")
        else:
            velocity = 0.3   # Set the desired velocity
            force = 0.0      # Set the desired force
            print("opening the gripper...")
            success = self.call_open_gripper(velocity, force)
            if success:
                print("gripper opened")
            else:
                print("error occurred")

        # Create and populate the feedback message
        feedback = GripperCommandFeedback()
        feedback.position = self.gripper_feedback
        # Publish feedback to MoveIt
        self.gripper_action_server.publish_feedback(feedback)

        # Create and populate the result message for a successful action
        result = GripperCommandResult()
        # Set the appropriate result values
        if success:
            self.gripper_action_server.set_succeeded(result)
        else:
            self.gripper_action_server.set_aborted(result)

        print("Done")

    def call_grasp_from_outside(self, velocity, force):
        rospy.wait_for_service('/panda/gripper/grasp_from_outside')
        try:
            grasp_from_outside = rospy.ServiceProxy('/panda/gripper/grasp_from_outside', SetGripperData)
            response = grasp_from_outside(velocity, force)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
            return False

    def call_open_gripper(self, velocity, force):
        rospy.wait_for_service('/panda/gripper/open')
        try:
            open_gripper = rospy.ServiceProxy('/panda/gripper/open', SetGripperData)
            response = open_gripper(velocity, force)
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: " + str(e))
            return False

    def joint_feedback(self, joints):
        self.joint_position_feedback = joints.position
        self.joint_velocity_feedback = joints.velocity
        self.joint_effort_feedback = joints.effort

    def gripper_feedback(self, gripper):
        self.gripper_feedback = gripper.data


if __name__ == '__main__':
    try:
        BridgeNode()
    except rospy.ROSInterruptException:
        pass
