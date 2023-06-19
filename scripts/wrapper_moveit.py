#!/usr/bin/env python3.8

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
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
        target_joint_topic_name = rospy.get_param('~target_joint_topic_name', 0)
        target_array_topic_name = rospy.get_param('~target_array_topic_name', 0)
        joint_state_topic_name = robot_name + '/joint_states'
        # Set up action server
        self.action_server = actionlib.SimpleActionServer(action_topic_name, FollowJointTrajectoryAction, self.execute_trajectory, False)
        self.action_server.start()

        # Initialize communication with your controller here
        self.target_joint_pub = rospy.Publisher(namespace + target_joint_topic_name, JointState, queue_size=1)
        self.target_array_pub = rospy.Publisher(robot_name + target_array_topic_name, Float64MultiArray, queue_size=1)
        self.joint_state_sub = rospy.Subscriber(joint_state_topic_name, JointState, self.joint_feedback)

        # Initialize messages
        self.target_joint = JointState()
        self.target_array = Float64MultiArray()

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

    def joint_feedback(self, joints):
        self.joint_position_feedback = joints.position
        self.joint_velocity_feedback = joints.velocity
        self.joint_effort_feedback = joints.effort


if __name__ == '__main__':
    try:
        BridgeNode()
    except rospy.ROSInterruptException:
        pass
