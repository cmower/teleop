#!/usr/bin/env python3
import rospy
import numpy as np
from custom_srvs.custom_srvs import ToggleService

class Node:

    def __init__(self):

        # Initialize ros node
        rospy.init_node('teleop_joint_mode')

        # Get parameters
        config = rospy.get_param('~config')
        self.joint_names = config['joint_names']
        self.hz = config.get('hz', 50)

        # Setup class attributes
        self.timer = None

        # Start toggle service
        ToggleService('toggle_joint_mode', self.start, self.stop)
        if rospy.get_param('~start_on_init', False):
            self.start()

    def start(self):
        if self.timer is None:
            self.timer = rospy.Duration()

    def stop(self):
        pass

    def callback(self, msg):
        self.h = np.array(msg.data)

    def main_loop(self, event):
        pass

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
