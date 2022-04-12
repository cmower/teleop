#!/usr/bin/env python3
import rospy
import numpy as np
from custom_ros_tools.tf import TfInterface
from custom_ros_tools.ros_comm import ToggleService

x0 = np.array([1., 0., 0.])
y0 = np.array([0., 1., 0.])
z0 = np.array([0., 0., 1.])

class Node:

    def __init__(self):

        # Initialize ROS node
        rospy.init_node('adaptive_orientation_control_node')

        # Get parameters
        self.parent = rospy.get_param('~parent_frame_id')
        self.child = rospy.get_param('~child_frame_id')
        self.adaptive = rospy.get_param('~adaptive_frame_id')
        self.alpha_min = rospy.get_param('~alpha_min', 0.01)
        self.dt = rospy.Duration(1.0/float(rospy.get_param('~hz', 100)))

        # Setup tf interface
        self.tf = TfInterface()

        # Setup toggle service
        self.timer = None
        ToggleService('adaptive_tool_frame_rotation/toggle', self.start, self.stop)
        if rospy.get_param('~start_on_init', False):
            self.start()

    def start(self):
        if self.timer is None:
            self.timer = rospy.Timer(self.dt, self.main_loop)
            success = True
            message = 'started adaptive tool frame rotation node'
        else:
            success = False
            message = 'failed to start adaptive tool frame rotation node, it is already running'
        return success, message

    def stop(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None
            success = True
            message = 'stopped adaptive tool frame rotation node'
        else:
            success = False
            message = 'failed to stop adaptive tool frame rotation node, it is not running'
        return success, message

    def main_loop(self, event):

        # Get transform
        tf = self.tf.get_tf_msg(self.parent, self.child)
        if tf is None: return

        # Convert tf to matrix
        T = self.tf.msg_to_matrix(tf)

        # Extract unit vectors from T
        x1 = T[:3, 0].flatten()
        y1 = T[:3, 1].flatten()
        z1 = T[:3, 2].flatten()

        # Assign z2 as z1
        z2 = z1.copy()

        # Compute alpha
        alpha = np.arccos(z0.dot(z2))

        # Compute x2
        if alpha >= self.alpha_min:
            z0_x_z2 = np.cross(z0, z2)
            x2 = z0_x_z2 / np.linalg.norm(z0_x_z2)
        else:
            x2 = x0.copy()

        # Compute y2
        y2 = np.cross(z2, x2)

        # Put together adaptive transform
        Tnew = T.copy()
        Tnew[:3, 0] = x2
        Tnew[:3, 1] = y2
        Tnew[:3, 2] = z2

        # Set tf
        self.tf.set_matrix(self.parent, self.adaptive, Tnew)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
