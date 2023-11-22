#!/usr/bin/env python

# 时间：20231024
# 功能：创建一个节点，订阅一个话题，在回调函数中 对接收到的/walker/leftLimb/joint_states话题数据进行处理，
# 然后对数据进行处理转发发布到另一个话题/joint_command。

import rospy
from sensor_msgs.msg import JointState
import numpy as np
# import pandas as pd

left_limb_j = JointState()


def update_joint_data(data):
    # 在此將ROS話題中的內容賦值給另一个变量，以发布至joint_command中
    # region 注释变量区
    # global left_limb_j1, left_limb_j2, left_limb_j3, left_limb_j4, left_limb_j5, left_limb_j6, left_limb_j7 # noqa
    # left_limb_j = [left_limb_j1,left_limb_j2,left_limb_j3,left_limb_j4,left_limb_j5,left_limb_j6,left_limb_j7] # noqa

    # global body_to_lhipyaw, body_to_rhipyaw, headpitch, left_limb_j1, right_limb_j1, lhipyaw_to_lhiproll, rhipyaw_to_rhiproll, headroll # noqa
    # global left_limb_j2, right_limb_j2, lhiproll_to_lhippitch, rhiproll_to_rhippitch, headyaw, left_limb_j3, right_limb_j3, lhippitch_to_lkneepitch # noqa
    # global rhippitch_to_rkneepitch, left_limb_j4, right_limb_j4, lkneepitch_to_lanklepitch, rkneepitch_to_ranklepitch, left_limb_j5, right_limb_j5 # noqa
    # global lanklepitch_to_lankleroll, ranklepitch_to_rankleroll, left_limb_j6, right_limb_j6, left_limb_j7, right_limb_j7, left_index_j1, left_middle_j1 # noqa
    # global left_pinky_j1, left_ring_j1, left_thumb_j1, right_index_j1, right_middle_j1, right_pinky_j1, right_ring_j1, right_thumb_j1, left_index_j2 # noqa
    # global left_middle_j2, left_pinky_j2, left_ring_j2, left_thumb_j2, right_index_j2, right_middle_j2, right_pinky_j2, right_ring_j2, right_thumb_j2 # noqa
    # global left_thumb_j3, right_thumb_j3, left_thumb_j4, right_thumb_j4
    # endregion
    left_limb_j.position = data.position
    left_limb_j.velocity = data.velocity
    left_limb_j.effort = data.effort
    # region 关节角赋值区(默认position值)
    body_to_lhipyaw = 0.0
    body_to_rhipyaw = 0.0
    headpitch = 0.0
    left_limb_j1 = left_limb_j.position[0]  #
    right_limb_j1 = 0.0
    lhipyaw_to_lhiproll = 0.0
    rhipyaw_to_rhiproll = 0.0
    headroll = 0.0
    left_limb_j2 = left_limb_j.position[1]
    right_limb_j2 = 0.0
    lhiproll_to_lhippitch = 0.0
    rhiproll_to_rhippitch = 0.0
    headyaw = 0.0
    left_limb_j3 = left_limb_j.position[2]
    right_limb_j3 = 0.0
    lhippitch_to_lkneepitch = 0.0
    rhippitch_to_rkneepitch = 0.0
    left_limb_j4 = left_limb_j.position[3]
    right_limb_j4 = 0.0
    lkneepitch_to_lanklepitch = 0.0
    rkneepitch_to_ranklepitch = 0.0
    left_limb_j5 = left_limb_j.position[4]
    right_limb_j5 = 0.0
    lanklepitch_to_lankleroll = 0.0
    ranklepitch_to_rankleroll = 0.0
    left_limb_j6 = left_limb_j.position[5]
    right_limb_j6 = 0.0
    left_limb_j7 = left_limb_j.position[6]
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
    # 将关节角position拼接成数组
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
    joint_state.position = np.array([0.0] * num_joints)
    joint_state.position = set_joints
    # 将数据储存
    pass
    # 发布
    pub.publish(joint_state)


def transit():
    rospy.init_node('joint_state_transmitter', anonymous=True)
    global pub
    pub = rospy.Publisher('joint_command', JointState, queue_size=10)
    rospy.Subscriber('/walker/leftLimb/joint_states', JointState, update_joint_data) # noqa
    rospy.spin()  # 会停留在这里 一直订阅leftLimb_joint_states话题的内容


if __name__ == '__main__':
    # initializeParams()
    transit()
    print('end of main')
