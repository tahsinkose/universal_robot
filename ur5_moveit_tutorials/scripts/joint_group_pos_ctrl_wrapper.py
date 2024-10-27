#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import numpy as np
import moveit_commander
import sys

class JointGroupPositionControllerWrapper:
    def __init__(self):
        # Target joint order and positions
        moveit_commander.roscpp_initialize(sys.argv)
        group = moveit_commander.MoveGroupCommander("manipulator", wait_for_servers=60.0)
        self.ordered_target_joint_names = group.get_active_joints()
        rospy.loginfo("Available joints: %s", self.ordered_target_joint_names)
        self.target_joint_positions = np.array([-5.62, -1.785, 1.625, -2.737, 5.707, -3.92])
        self.current_joint_positions = None
        self.tolerance = 0.01

        # Publisher for the target joint positions
        self.pub = rospy.Publisher(
            '/joint_group_pos_controller/command', Float64MultiArray, queue_size=10)

        # Subscriber for the joint states
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, msg):
        """
        Callback function to update the current joint positions
        when a message is received on the /joint_states topic.
        """
        joint_position_dict = dict(zip(msg.name, msg.position))
        self.current_joint_positions = np.array(
            [joint_position_dict[joint] for joint in self.ordered_target_joint_names])

    def publish_joint_positions(self):
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Create and publish a Float64MultiArray message with the target positions
            msg = Float64MultiArray()
            msg.data = self.target_joint_positions.tolist()
            rospy.logdebug("Publishing target joint positions: %s", msg.data)
            self.pub.publish(msg)

            # Check if the current joint positions are close to the target positions
            if self.current_joint_positions is not None:
                rospy.logdebug("Current joint positions: %s", self.current_joint_positions)

                if np.allclose(self.current_joint_positions, self.target_joint_positions, atol=self.tolerance):
                    rospy.loginfo("Target joint positions reached. Exiting...")
                    break

            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('joint_group_position_controller_wrapper', anonymous=True)
        controller = JointGroupPositionControllerWrapper()
        controller.publish_joint_positions()
    except rospy.ROSInterruptException:
        pass
