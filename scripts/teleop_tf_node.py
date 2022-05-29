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
import sys
import rospy
import numpy as np
import tf_conversions
from std_msgs.msg import Float64MultiArray
from custom_ros_tools.tf import TfInterface
from custom_srvs.srv import SetTransform, SetTransformResponse
from custom_ros_tools.ros_comm import ToggleService
from std_srvs.srv import Trigger, TriggerResponse
from teleop.srv import GetBoxLimits, GetBoxLimitsResponse

class Node:

    def __init__(self):

        # Initialize ros node
        rospy.init_node('teleop_tf_node')
        self.tf = TfInterface()

        # Get parameters
        config = rospy.get_param('~config')
        self.parent_frame = config.get('parent_frame_id', 'world')
        self.child_frame = config['target_frame_id']
        self.dt = 1.0/float(config.get('hz', 100))
        self.duration = rospy.Duration(self.dt)

        xlim = config.get('xlim', [-np.inf, np.inf])
        ylim = config.get('ylim', [-np.inf, np.inf])
        zlim = config.get('zlim', [-np.inf, np.inf])
        self.xlim = np.array(xlim)
        self.ylim = np.array(ylim)
        self.zlim = np.array(zlim)
        self.lolim = np.array([xlim[0], ylim[0], zlim[0]])
        self.uplim = np.array([xlim[1], ylim[1], zlim[1]])
        if not self.is_in_limit([0,0,0]):
            rospy.logerr('origin is not inside the specified box limits!')
            sys.exit(0)

        # Setup other class attributes
        self.h = np.zeros(6)
        self.transform = np.zeros(6)  # position + euler angles
        self.timer = None

        # Setup ros comm
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)
        ToggleService('toggle_teleop_tf', self.start_teleop, self.stop_teleop)
        rospy.Service('reset_teleop_transform', SetTransform, self.reset_transform)
        rospy.Service('reset_teleop_transform_to_zero', Trigger, self.reset_zero)
        rospy.Service('teleop_box_limits', GetBoxLimits, self.get_box_limits)

        # Start on initialization (optional)
        if rospy.get_param('~start_on_init', False):
            self.start_teleop()

    def get_box_limits(self, req):
        return GetBoxLimitsResponse(
            xlim=self.xlim.tolist(),
            ylim=self.xlim.tolist(),
            zlim=self.xlim.tolist(),
        )

    def reset_transform(self, req):
        pos = self.tf.tf_msg_to_pos(req.transform)
        if self.is_in_limit(pos):
            rot = self.tf.tf_msg_to_eul(req.transform)
            self.transform = np.concatenate((pos, rot))
            self.set_tf()
            return SetTransformResponse(success=True, message='reset transform to ' + str(self.transform))
        else:
            return SetTransformResponse(success=False, message='given transform is outside limits!')

    def reset_zero(self, req):
        self.transform = np.zeros(6)
        self.set_tf()
        return TriggerResponse(success=True, message='reset transform to zero')

    def start_teleop(self):
        if self.timer is None:
            self.timer = rospy.Timer(self.duration, self.main_loop)
            success = True
            message = 'started tf teleoperation node'
        else:
            success = False
            message = 'user attempted to start tf teleoperation node, but it is already running!'
            rospy.logerr(message)
        return success, message

    def stop_teleop(self):
        if self.timer is not None:
            self.h = np.zeros(6)  # ensure no more motion
            self.timer.shutdown()
            self.timer = None
            success = True
            message = 'stopped tf teleoperation node'
        else:
            success = False
            message = 'user attempted to stop tf teleoperation node, but it is not running!'
        return success, message

    def callback(self, msg):
        n = len(msg.data)
        if n == 3:
            # update only position
            self.h[:3] = np.array(msg.data)
        elif n == 6:
            # update position and rotation
            self.h[:6] = np.array(msg.data)
        else:
            raise ValueError(f"recieved operator signal is not correct length, expected 3 or 6, got {n}")

    def is_in_limit(self, pos):
        return np.logical_and(self.lolim <= pos, pos <= self.uplim).all()

    def clip_transform(self, transform):
        transform[:3] = np.clip(transform[:3], self.lolim, self.uplim)
        return transform

    def apply_user_input(self):
        self.transform = self.clip_transform(self.transform + self.dt*self.h)

    def main_loop(self, event):
        self.apply_user_input()
        self.set_tf()

    def set_tf(self):
        self.tf.set_tf(
            self.parent_frame,
            self.child_frame,
            self.transform[:3],
            tf_conversions.transformations.quaternion_from_euler(self.transform[3], self.transform[4], self.transform[5])
        )

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
