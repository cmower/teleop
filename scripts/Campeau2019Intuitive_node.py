#!/usr/bin/env python3
import rospy
# import tf2_ros
import numpy as np
import tf_conversions
from std_msgs.msg import Float64MultiArray
# from geometry_msgs.msg import TransformStamped
from ros_pybullet_interface.tf_interface import TfInterface

"""

My implementation of [1].

References
----------

 [1] Alexandre Campeau-Lecours, Ulysse Côté-Allard, Dinh-Son Vu,
     François Routhier, Benoit Gosselin, and Clément Gosselin,
     Intuitive Adaptive Orientation Control for Enhanced Human–Robot
     Interaction, IEEE Transactions On Robotics, Vol. 35, No. 2, April
     2019.


"""

ex0 = np.array([1., 0., 0.])
ey0 = np.array([0., 1., 0.])
ez0 = np.array([0., 0., 1.])

class Node:

    def __init__(self):

        # Init ros node
        rospy.init_node('adaptive_orientation_node')

        # Setup tf interface
        self.tf = TfInterface()

        # Get parameters
        self.world_frame = rospy.get_param('~world_frame')
        self.eff_frame = rospy.get_param('~eff_frame')
        self.goal_frame = rospy.get_param('~goal_frame')
        self.alpha_min = np.clip(rospy.get_param('~alpha_min', 0.001), 0.0, np.inf)
        hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(hz)

        # Subscribe to operator signal messages
        self.h = np.zeros(6)
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)

        # Start main loop
        rospy.Timer(rospy.Duration(self.dt), self.main_loop)

    def callback(self, msg):
        self.h = np.array(msg.data[:6])

    def main_loop(self, event):

        # Get transform
        pos, rot = self.tf.get_tf(self.world_frame, self.eff_frame)
        if pos is None: return

        # Get eff transform unit vectors
        T = tf_conversions.transformations.quaternion_matrix(rot)
        # ex1, ey1, ez1 = np.hsplit(T[:3,:], 3)
        ex1 = T[:3,0]
        ey1 = T[:3,1]
        ez1 = T[:3,2]

        # Compute new transformation matrix. See [1, Sec II(C)].
        ez2 = ez1.copy()
        alpha = np.arccos(ez0.dot(ez2))
        if alpha >= self.alpha_min:
            ez0_x_ez2 = np.cross(ez0, ez2)
            ex2 = ez0_x_ez2/np.linalg.norm(ez0_x_ez2)
        else:
            ex2 = ex0.copy()
        ey2 = np.cross(ez2, ex2)

        # Update transformation with user input
        d = self.dt*self.h.copy()
        p = np.array(pos)
        R = np.zeros((3, 3))
        R[:,0] = ex2
        R[:,1] = ey2
        R[:,2] = ez2
        new_pos = p + R@d[:3]
        eul = tf_conversions.transformations.euler_from_matrix(R)
        new_eul = eul + d[3:]
        new_quat = tf_conversions.transformations.quaternion_from_euler(*new_eul.tolist())

        # Set new transformation
        self.tf.set_tf(self.world_frame, self.goal_frame, new_pos, new_quat)


    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
