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
import tf_conversions
from std_msgs.msg import Float64MultiArray
from custom_ros_tools.tf import TfInterface
from custom_srvs.srv import SetTransform, SetTransformResponse
from custom_ros_tools.ros_comm import ToggleService
from std_srvs.srv import Trigger, TriggerResponse

class Node:

    def __init__(self):
        rospy.init_node('teleop_tf_node')
        self.tf = TfInterface()
        self.parent_frame = rospy.get_param('~parent_frame_id', 'world')
        self.child_frame = rospy.get_param('~child_frame_id')
        hz = rospy.get_param('~hz', 100)
        self.dt = 1.0/float(hz)
        self.h = np.zeros(6)
        self.transform = np.zeros(6)  # position + euler angles
        self.timer = None
        rospy.Subscriber('operator_node/signal', Float64MultiArray, self.callback)
        ToggleService('toggle_teleop_tf', self.start_teleop, self.stop_teleop)
        rospy.Service('reset_transform', SetTransform, self.reset_transform)
        rospy.Service('reset_zero', Trigger, self.reset_zero)
        if rospy.get_param('~start_on_init', False):
            self.start_teleop()

    def reset_transform(self, req):
        self.transform[:3] = np.array([getattr(req.transform.translation, dim) for dim in 'xyz'])
        rot = np.array([getattr(req.transform.rotation, dim) for dim in 'xyzw'])
        self.transform[3:] = tf_conversions.transformations.euler_from_quaternion(rot)
        return SetTransformResponse(success=True, message='reset transform to ' + str(self.transform))

    def reset_zero(self, req):
        self.transform = np.zeros(6)
        return TriggerResponse(success=True, message='reset transform to zero')

    def start_teleop(self):
        if self.timer is None:
            self.timer = rospy.Timer(rospy.Duration(self.dt), self.main_loop)
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

    def main_loop(self, event):
        self.transform += self.dt*self.h
        self.tf.set_tf(self.parent_frame, self.child_frame, self.transform[:3], tf_conversions.transformations.quaternion_from_euler(self.transform[3], self.transform[4], self.transform[5]))

    def spin(self):
        rospy.spin()


def main():
    Node().spin()


if __name__ == '__main__':
    main()
