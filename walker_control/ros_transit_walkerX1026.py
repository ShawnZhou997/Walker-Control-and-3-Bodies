#!/usr/bin/env python

# 时间：20231026
# 功能：创建一个节点，订阅一个话题，在回调函数中 对接收到的/walker/leftLimb/joint_states话题数据进行处理，
# 然后对数据进行处理转发发布到另一个话题/joint_command。
# 1026目标新增：加入其他话题的内容

import rospy
from sensor_msgs.msg import JointState
import numpy as np


def initializeParams():
    global counter, joint_num  # 用于计算是否所有的回调函数都被调用完毕，其关节是否都得到更新
    counter = 0
    joint_num = 2  # 主端发来的关节组数
    global left_limb_j
    left_limb_j = JointState()
    global right_limb_j
    right_limb_j = JointState()
    # region 关节角赋值区(默认position值)
    global body_to_lhipyaw, body_to_rhipyaw, headpitch, left_limb_j1, right_limb_j1, lhipyaw_to_lhiproll, rhipyaw_to_rhiproll, headroll # noqa
    global left_limb_j2, right_limb_j2, lhiproll_to_lhippitch, rhiproll_to_rhippitch, headyaw, left_limb_j3, right_limb_j3, lhippitch_to_lkneepitch # noqa
    global rhippitch_to_rkneepitch, left_limb_j4, right_limb_j4, lkneepitch_to_lanklepitch, rkneepitch_to_ranklepitch, left_limb_j5, right_limb_j5 # noqa
    global lanklepitch_to_lankleroll, ranklepitch_to_rankleroll, left_limb_j6, right_limb_j6, left_limb_j7, right_limb_j7, left_index_j1, left_middle_j1 # noqa
    global left_pinky_j1, left_ring_j1, left_thumb_j1, right_index_j1, right_middle_j1, right_pinky_j1, right_ring_j1, right_thumb_j1, left_index_j2 # noqa
    global left_middle_j2, left_pinky_j2, left_ring_j2, left_thumb_j2, right_index_j2, right_middle_j2, right_pinky_j2, right_ring_j2, right_thumb_j2 # noqa
    global left_thumb_j3, right_thumb_j3, left_thumb_j4, right_thumb_j4
    body_to_lhipyaw = 0.0
    body_to_rhipyaw = 0.0
    headpitch = 0.0
    left_limb_j1 = 0.0
    right_limb_j1 = 0.0
    lhipyaw_to_lhiproll = 0.0
    rhipyaw_to_rhiproll = 0.0
    headroll = 0.0
    left_limb_j2 = 0.0
    right_limb_j2 = 0.0
    lhiproll_to_lhippitch = 0.0
    rhiproll_to_rhippitch = 0.0
    headyaw = 0.0
    left_limb_j3 = 0.0
    right_limb_j3 = 0.0
    lhippitch_to_lkneepitch = 0.0
    rhippitch_to_rkneepitch = 0.0
    left_limb_j4 = 0.0
    right_limb_j4 = 0.0
    lkneepitch_to_lanklepitch = 0.0
    rkneepitch_to_ranklepitch = 0.0
    left_limb_j5 = 0.0
    right_limb_j5 = 0.0
    lanklepitch_to_lankleroll = 0.0
    ranklepitch_to_rankleroll = 0.0
    left_limb_j6 = 0.0
    right_limb_j6 = 0.0
    left_limb_j7 = 0.0
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
    global set_joints
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
    # endregion


def update_leftLimb_data(data):
    global left_limb_j1, left_limb_j2, left_limb_j3, left_limb_j4, left_limb_j5, left_limb_j6, left_limb_j7 # noqa
    left_limb_j.position = data.position
    left_limb_j.velocity = data.velocity
    left_limb_j.effort = data.effort
    left_limb_j1 = left_limb_j.position[0]
    left_limb_j2 = left_limb_j.position[1]
    left_limb_j3 = left_limb_j.position[2]
    left_limb_j4 = left_limb_j.position[3]
    left_limb_j5 = left_limb_j.position[4]
    left_limb_j6 = left_limb_j.position[5]
    left_limb_j7 = left_limb_j.position[6]
    global set_joints
    set_joints[3] = left_limb_j1
    set_joints[8] = left_limb_j2
    set_joints[13] = left_limb_j3
    set_joints[17] = left_limb_j4
    set_joints[21] = left_limb_j5
    set_joints[25] = left_limb_j6
    set_joints[27] = left_limb_j7
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_rightLimb_data(data):
    global right_limb_j1, right_limb_j2, right_limb_j3, right_limb_j4, right_limb_j5, right_limb_j6, right_limb_j7 # noqa
    right_limb_j.position = data.position
    right_limb_j.velocity = data.velocity
    right_limb_j.effort = data.effort
    right_limb_j1 = right_limb_j.position[0]
    right_limb_j2 = right_limb_j.position[1]
    right_limb_j3 = right_limb_j.position[2]
    right_limb_j4 = right_limb_j.position[3]
    right_limb_j5 = right_limb_j.position[4]
    right_limb_j6 = right_limb_j.position[5]
    right_limb_j7 = right_limb_j.position[6]
    global set_joints
    set_joints[4] = right_limb_j1
    set_joints[9] = right_limb_j2
    set_joints[14] = right_limb_j3
    set_joints[18] = right_limb_j4
    set_joints[22] = right_limb_j5
    set_joints[26] = right_limb_j6
    set_joints[28] = right_limb_j7
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def publishJoint():
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
    global set_joints
    joint_state.position = set_joints
    # 将数据储存
    pass
    # 发布
    pub.publish(joint_state)
    global counter
    counter = 0


def transit():
    rospy.init_node('joint_state_transmitter', anonymous=True)
    global pub
    pub = rospy.Publisher('joint_command', JointState, queue_size=10)
    rospy.Subscriber('/walker/leftLimb/joint_states', JointState, update_leftLimb_data) # noqa
    rospy.Subscriber('/walker/rightLimb/joint_states', JointState, update_rightLimb_data) # noqa
    rospy.spin()  # 会停留在这里 一直订阅leftLimb_joint_states话题的内容


if __name__ == '__main__':
    initializeParams()
    transit()
