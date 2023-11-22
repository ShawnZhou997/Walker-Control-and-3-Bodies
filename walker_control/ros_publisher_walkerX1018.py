#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np
import time

rospy.init_node("Transit", anonymous=True)

isNotTransit = False  # True表示用脚本中的常量控制，False表示从外部ROS接收
# if not isNotTransit:
#     left_limb_j = JointState()
left_limb_j = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pub = rospy.Publisher("/joint_command", JointState, queue_size=10)
joint_state = JointState()


def refreshLeftLimbJntStateData(data):
    # 在此將ROS話題中的內容賦值給全局變量
    global left_limb_j
    # global left_limb_j1, left_limb_j2, left_limb_j3, left_limb_j4, left_limb_j5, left_limb_j6, left_limb_j7 # noqa
    # left_limb_j = [left_limb_j1,left_limb_j2,left_limb_j3,left_limb_j4,left_limb_j5,left_limb_j6,left_limb_j7] # noqa

    # global body_to_lhipyaw, body_to_rhipyaw, headpitch, left_limb_j1, right_limb_j1, lhipyaw_to_lhiproll, rhipyaw_to_rhiproll, headroll # noqa
    # global left_limb_j2, right_limb_j2, lhiproll_to_lhippitch, rhiproll_to_rhippitch, headyaw, left_limb_j3, right_limb_j3, lhippitch_to_lkneepitch # noqa
    # global rhippitch_to_rkneepitch, left_limb_j4, right_limb_j4, lkneepitch_to_lanklepitch, rkneepitch_to_ranklepitch, left_limb_j5, right_limb_j5 # noqa
    # global lanklepitch_to_lankleroll, ranklepitch_to_rankleroll, left_limb_j6, right_limb_j6, left_limb_j7, right_limb_j7, left_index_j1, left_middle_j1 # noqa
    # global left_pinky_j1, left_ring_j1, left_thumb_j1, right_index_j1, right_middle_j1, right_pinky_j1, right_ring_j1, right_thumb_j1, left_index_j2 # noqa
    # global left_middle_j2, left_pinky_j2, left_ring_j2, left_thumb_j2, right_index_j2, right_middle_j2, right_pinky_j2, right_ring_j2, right_thumb_j2 # noqa
    # global left_thumb_j3, right_thumb_j3, left_thumb_j4, right_thumb_j4
    print("refreshLeftLimbJntStateData")
    left_limb_j.position = data.position
    left_limb_j.velocity = data.velocity
    left_limb_j.effort = data.effort


def listener():
    print("listener")
    rospy.Subscriber('leftLimb_joint_states', JointState, refreshLeftLimbJntStateData) # JointState消息作为参数传递给refreshData函数 # noqa
    print("listener mid")
    rospy.spin()
    print("listener end")


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

# region 变量赋值区_默认值
if isNotTransit:
    left_limb_j = [30, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    left_limb_j1 = left_limb_j[0]  #
    left_limb_j2 = left_limb_j[1]
    left_limb_j3 = left_limb_j[2]
    left_limb_j4 = left_limb_j[3]
    left_limb_j5 = left_limb_j[4]
    left_limb_j6 = left_limb_j[5]
    left_limb_j7 = left_limb_j[6]
else:
    pass  # 给left_limb_j赋值的句子在refresh...函数中将会被执行
body_to_lhipyaw = 0.0
body_to_rhipyaw = 0.0
headpitch = 0.0
right_limb_j1 = 30.0
lhipyaw_to_lhiproll = 0.0
rhipyaw_to_rhiproll = 0.0
headroll = 0.0
right_limb_j2 = 0.0
lhiproll_to_lhippitch = 0.0
rhiproll_to_rhippitch = 0.0
headyaw = 0.0
right_limb_j3 = 0.0
lhippitch_to_lkneepitch = 0.0
rhippitch_to_rkneepitch = 0.0
right_limb_j4 = 0.0
lkneepitch_to_lanklepitch = 0.0
rkneepitch_to_ranklepitch = 0.0
right_limb_j5 = 0.0
lanklepitch_to_lankleroll = 0.0
ranklepitch_to_rankleroll = 0.0
right_limb_j6 = 0.0
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
if isNotTransit:
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
    default_joints = np.radians(set_joints)

# region 变量对应关系归类区
# # 双腿
# LHipYaw = body_to_lhipyaw
# LHipRoll = lhipyaw_to_lhiproll
# LHipPitch = lhiproll_to_lhippitch
# LKneePitch = lhippitch_to_lkneepitch
# LAnklePitch = lkneepitch_to_lanklepitch
# LAnkleRoll = lhipyaw_to_lhiproll
# RHipYaw = body_to_rhipyaw
# RHipRoll = rhipyaw_to_rhiproll
# RHipPitch = rhiproll_to_rhippitch
# RKneePitch = rhippitch_to_rkneepitch
# RAnklePitch = rkneepitch_to_ranklepitch
# RAnkleRoll = ranklepitch_to_rankleroll
# leg_MeasuredJoint = [LHipYaw, LHipRoll, LHipPitch,
#                      LKneePitch, LAnklePitch, LAnkleRoll,
#                      RHipYaw, RHipRoll, RHipPitch,
#                      RKneePitch, RAnklePitch, RAnkleRoll]
# # 头 walker2和walkerX差一个headRoll
# HeadYaw = headyaw
# HeadPitch = headpitch
# head_joint_states = [HeadYaw, HeadPitch]
# # 左掌：1. Thumb（拇指）2. Index finger（食指）3. Middle finger（中指）4. Ring finger（无名指）5. Little finger（小指） # noqa
# # walker2和walkerX拇指差2个关节
# LFirstFinger1 = left_thumb_j1
# LFirstFinger2 = left_thumb_j2
# LSecondFinger1 = left_index_j1
# LSecondFinger2 = left_index_j2
# LThirdFinger1 = left_middle_j1
# LThirdFinger2 = left_middle_j2
# LForthFinger1 = left_ring_j1
# LForthFinger2 = left_ring_j2
# LFifthFinger1 = left_pinky_j1
# LFifthFinger2 = left_pinky_j2
# leftHand_joint_states = [LFirstFinger1, LFirstFinger2,
#                          LSecondFinger1, LSecondFinger2,
#                          LThirdFinger1, LThirdFinger2,
#                          LForthFinger1, LForthFinger2,
#                          LFifthFinger1, LFifthFinger2]
# # 左臂
# LShoulderPitch = left_limb_j1
# LShoulderRoll = left_limb_j2
# LShoulderYaw = left_limb_j3
# LElbowRoll = left_limb_j4
# LElbowYaw = left_limb_j5
# LWristPitch = left_limb_j6
# LWristRoll = left_limb_j7
# leftLimb_joint_states = [LShoulderPitch, LShoulderRoll,
#                          LShoulderYaw, LElbowRoll,
#                          LElbowYaw, LWristPitch, LWristRoll]
# # 右掌
# RFirstFinger1 = right_thumb_j1
# RFirstFinger2 = right_thumb_j2
# RSecondFinger1 = right_index_j1
# RSecondFinger2 = right_index_j2
# RThirdFinger1 = right_middle_j1
# RThirdFinger2 = right_middle_j2
# RForthFinger1 = right_ring_j1
# RForthFinger2 = right_ring_j2
# RFifthFinger1 = right_pinky_j1
# RFifthFinger2 = right_pinky_j2
# rightHand_joint_states = [RFirstFinger1, RFirstFinger2,
#                           RSecondFinger1, RSecondFinger2,
#                           RThirdFinger1, RThirdFinger2,
#                           RForthFinger1, RForthFinger2,
#                           RFifthFinger1, RFifthFinger2]
# # 右臂
# RShoulderPitch = right_limb_j1
# RShoulderRoll = right_limb_j2
# RShoulderYaw = right_limb_j3
# RElbowRoll = right_limb_j4
# RElbowYaw = right_limb_j5
# RWristPitch = right_limb_j6
# RWristRoll = right_limb_j7
# rightLimb_joint_states = [RShoulderPitch, RShoulderRoll,
#                           RShoulderYaw, RElbowRoll,
#                           RElbowYaw, RWristPitch, RWristRoll]

# endregion

# position control the robot to wiggle around each joint
time_start = time.time()
rate = rospy.Rate(20)

if isNotTransit:
    while not rospy.is_shutdown():
        joint_state.position = default_joints
        pub.publish(joint_state)
        rate.sleep()
else:
    listener()
    set_joints = [
        body_to_lhipyaw,
        body_to_rhipyaw,
        headpitch,
        left_limb_j[0],
        right_limb_j1,
        lhipyaw_to_lhiproll,
        rhipyaw_to_rhiproll,
        headroll,
        left_limb_j[1],
        right_limb_j2,
        lhiproll_to_lhippitch,
        rhiproll_to_rhippitch,
        headyaw,
        left_limb_j[2],
        right_limb_j3,
        lhippitch_to_lkneepitch,
        rhippitch_to_rkneepitch,
        left_limb_j[3],
        right_limb_j4,
        lkneepitch_to_lanklepitch,
        rkneepitch_to_ranklepitch,
        left_limb_j[4],
        right_limb_j5,
        lanklepitch_to_lankleroll,
        ranklepitch_to_rankleroll,
        left_limb_j[5],
        right_limb_j6,
        left_limb_j[6],
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
    # new_joints = np.radians(set_joints)
    # joint_state.position = new_joints
    joint_state.position = set_joints
    pub.publish(joint_state)
    rate.sleep()
