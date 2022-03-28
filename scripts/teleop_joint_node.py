#!/usr/bin/env python3
# BSD 2-Clause License
# Copyright (c) 2022, Christopher E. Mower
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class Node:

    def __init__(self):

        # Initialize node
        rospy.init_node('teleop_joint_node')

        # Init variables
        self.q = None

        # Get parameters
        hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(hz)
        self.joint_names = rospy.get_param('~joint_names')
        control_mode = rospy.get_param('~control_mode', 'position')  # position/velocity
        if control_mode == 'position':
            curr_joint_state = rospy.wait_for_message('joint_states', JointState)
            self.q = np.array([curr_joint_state.position[curr_joint_state.name.index(n)] for n in self.joint_names])

        # Setup publisher
        self.pub = rospy.Publisher('joint_states/target', JointState, queue_size=10)

        # Start subscriber
        self.h = np.zeros(len(self.joint_names))
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)

        # Start timer
        if control_mode == 'position':
            rospy.Timer(rospy.Duration(self.dt), self.main_loop_position)
        elif control_mode == 'velocity':
            rospy.Timer(rospy.Duration(self.dt), self.main_loop_velocity)
        else:
            raise ValueError(f'did not recognize control mode ({control_mode})!')


    def callback(self, msg):
        self.h = np.array(msg.data)

    def main_loop_position(self, event):
        self.q += self.dt*self.h
        msg = JointState(name=self.joint_names, position=self.q.tolist())
        self.pub.publish(msg)

    def main_loop_velocity(self, event):
        msg = JointState(name=self.joint_names, velocity=self.h.tolist())
        self.pub.publish(msg)

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
