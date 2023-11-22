#!/usr/bin/env python
# 此脚本可以驱动walkerX抬双手

# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("test_rosbridge", anonymous=True)

pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
joint_state = JointState()


joint_state.name = [
    "body_to_lhipyaw",
    "body_to_rhipyaw",
    "headpitch",
    "left_limb_j1",
    "right_limb_j1",
    "lhipyaw_to_lhiproll",
    "rhipyaw_to_rhiproll",
    "headroll",
    "left_limb_j2",
    "right_limb_j2",
    "lhiproll_to_lhippitch",
    "rhiproll_to_rhippitch",
    "headyaw",
    "left_limb_j3",
    "right_limb_j3",
    "lhippitch_to_lkneepitch",
    "rhippitch_to_rkneepitch",
    "left_limb_j4",
    "right_limb_j4",
    "lkneepitch_to_lanklepitch",  # noqa
    "rkneepitch_to_ranklepitch",
    "left_limb_j5",
    "right_limb_j5",
    "lanklepitch_to_lankleroll",
    "ranklepitch_to_rankleroll",
    "left_limb_j6",
    "right_limb_j6",
    "left_limb_j7",
    "right_limb_j7",
    "left_index_j1",
    "left_middle_j1",
    "left_pinky_j1",
    "left_ring_j1",
    "left_thumb_j1",
    "right_index_j1",
    "right_middle_j1",
    "right_pinky_j1",
    "right_ring_j1",
    "right_thumb_j1",
    "left_index_j2",
    "left_middle_j2",
    "left_pinky_j2",
    "left_ring_j2",
    "left_thumb_j2",
    "right_index_j2",
    "right_middle_j2",
    "right_pinky_j2",
    "right_ring_j2",
    "right_thumb_j2",
    "left_thumb_j3",
    "right_thumb_j3",
    "left_thumb_j4",
    "right_thumb_j4",
]

num_joints = len(joint_state.name)

# make sure kit's editor is playing for receiving messages # noqa

joint_state.position = np.array([0.0] * num_joints)

left_limb_j = [30, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# region 变量赋值区
# 改进方向：分门别类地拼接字符串
body_to_lhipyaw = 0.0
body_to_rhipyaw = 0.0
headpitch = 0.0
left_limb_j1 = left_limb_j[0]  #
right_limb_j1 = 30.0
lhipyaw_to_lhiproll = 0.0
rhipyaw_to_rhiproll = 0.0
headroll = 0.0
left_limb_j2 = left_limb_j[1]
right_limb_j2 = 0.0
lhiproll_to_lhippitch = 0.0
rhiproll_to_rhippitch = 0.0
headyaw = 0.0
left_limb_j3 = left_limb_j[2]
right_limb_j3 = 0.0
lhippitch_to_lkneepitch = 0.0
rhippitch_to_rkneepitch = 0.0
left_limb_j4 = left_limb_j[3]
right_limb_j4 = 0.0
lkneepitch_to_lanklepitch = 0.0
rkneepitch_to_ranklepitch = 0.0
left_limb_j5 = left_limb_j[4]
right_limb_j5 = 0.0
lanklepitch_to_lankleroll = 0.0
ranklepitch_to_rankleroll = 0.0
left_limb_j6 = left_limb_j[5]
right_limb_j6 = 0.0
left_limb_j7 = left_limb_j[6]
right_limb_j7 = 0.0
left_index_j1 = 0.0
left_middle_j1 = 0.0
left_pinky_j1 = 0.0
left_ring_j1 = 0.0
left_thumb_j1 = 0.0
right_index_j1 = 0.0
right_middle_j1 = 0.0
right_pinky_j1 = 0.0
right_ring_j1 = 0.0
right_thumb_j1 = 0.0
left_index_j2 = 0.0
left_middle_j2 = 0.0
left_pinky_j2 = 0.0
left_ring_j2 = 0.0
left_thumb_j2 = 0.0
right_index_j2 = 0.0
right_middle_j2 = 0.0
right_pinky_j2 = 0.0
right_ring_j2 = 0.0
right_thumb_j2 = 0.0
left_thumb_j3 = 0.0
right_thumb_j3 = 0.0
left_thumb_j4 = 0.0
right_thumb_j4 = 0.0
# endregion
set_joints = [
    body_to_lhipyaw,
    body_to_rhipyaw,
    headpitch,
    left_limb_j1,
    right_limb_j1,
    lhipyaw_to_lhiproll,
    rhipyaw_to_rhiproll,
    headroll,
    left_limb_j2,
    right_limb_j2,
    lhiproll_to_lhippitch,
    rhiproll_to_rhippitch,
    headyaw,
    left_limb_j3,
    right_limb_j3,
    lhippitch_to_lkneepitch,
    rhippitch_to_rkneepitch,
    left_limb_j4,
    right_limb_j4,
    lkneepitch_to_lanklepitch,
    rkneepitch_to_ranklepitch,
    left_limb_j5,
    right_limb_j5,
    lanklepitch_to_lankleroll,
    ranklepitch_to_rankleroll,
    left_limb_j6,
    right_limb_j6,
    left_limb_j7,
    right_limb_j7,
    left_index_j1,
    left_middle_j1,
    left_pinky_j1,
    left_ring_j1,
    left_thumb_j1,
    right_index_j1,
    right_middle_j1,
    right_pinky_j1,
    right_ring_j1,
    right_thumb_j1,
    left_index_j2,
    left_middle_j2,
    left_pinky_j2,
    left_ring_j2,
    left_thumb_j2,
    right_index_j2,
    right_middle_j2,
    right_pinky_j2,
    right_ring_j2,
    right_thumb_j2,
    left_thumb_j3,
    right_thumb_j3,
    left_thumb_j4,
    right_thumb_j4,
]
default_joints = np.radians(set_joints)*3

# limiting the movements to a smaller range (this is not the range of the robot, just the range of the movement  # noqa
max_joints = np.array(default_joints) + 0.5
min_joints = np.array(default_joints) - 0.5

# position control the robot to wiggle around each joint
time_start = time.time()
rate = rospy.Rate(20)
while not rospy.is_shutdown():
    # joint_state.position = np.sin(time.time() - time_start) * (max_joints - min_joints) * 0.5 + default_joints # noqa
    # joint_state.position = default_joints #
    joint_state.position = default_joints
    pub.publish(joint_state)
    rate.sleep()
