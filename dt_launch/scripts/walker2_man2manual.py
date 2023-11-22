#!/usr/bin/env python

# 时间：20231030
# 功能：逆解话题转换成控制话题给IS的绿色机器人
# 1114：用于手动控制

import rospy
from sensor_msgs.msg import JointState
import numpy as np


def initializeParams():
    global counter, joint_num  # 用于计算是否所有的回调函数都被调用完毕，其关节是否都得到更新
    counter = 0
    joint_num = 2  # 主端发来的关节组数
    # 关节全局变量初始化
    global leg_j
    leg_j = JointState()
    global left_limb_j
    left_limb_j = JointState()
    global right_limb_j
    right_limb_j = JointState()
    global left_hand
    left_hand = JointState()
    global right_hand
    right_hand = JointState()
    global head
    head = JointState()
    # region 关节角赋值区(默认position值)
    global head_j1, left_leg_j1, left_limb_j1, right_leg_j1, right_limb_j1, head_j2, left_leg_j2, left_limb_j2, right_leg_j2, right_limb_j2, left_leg_j3, left_limb_j3, right_leg_j3, right_limb_j3, left_limb_j4, right_limb_j4, left_leg_j4, left_limb_j5, right_leg_j4, right_limb_j5, left_leg_j5, left_limb_j6, right_leg_j5, right_limb_j6, left_leg_j6, left_limb_j7, right_leg_j6, right_limb_j7, left_index_j1, left_middle_j1, left_pinky_j1, left_ring_j1, left_thumb_j1, right_index_j1, right_middle_j1, right_pinky_j1, right_ring_j1, right_thumb_j1, left_index_j2, left_middle_j2, left_pinky_j2, left_ring_j2, left_thumb_j2, right_index_j2, right_middle_j2, right_pinky_j2, right_ring_j2, right_thumb_j2 # noqa
    head_j1 = 0.0
    left_leg_j1 = 0.0
    left_limb_j1 = 0.0
    right_leg_j1 = 0.0
    right_limb_j1 = 0.0
    head_j2 = 0.0
    left_leg_j2 = 0.0
    left_limb_j2 = 0.0
    right_leg_j2 = 0.0
    right_limb_j2 = 0.0
    left_leg_j3 = 0.0
    left_limb_j3 = 0.0
    right_leg_j3 = 0.0
    right_limb_j3 = 0.0
    left_limb_j4 = 0.0
    right_limb_j4 = 0.0
    left_leg_j4 = 0.0
    left_limb_j5 = 0.0
    right_leg_j4 = 0.0
    right_limb_j5 = 0.0
    left_leg_j5 = 0.0
    left_limb_j6 = 0.0
    right_leg_j5 = 0.0
    right_limb_j6 = 0.0
    left_leg_j6 = 0.0
    left_limb_j7 = 0.0
    right_leg_j6 = 0.0
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
    global set_joints
    set_joints = [
        head_j1,
        left_leg_j1,
        left_limb_j1,
        right_leg_j1,
        right_limb_j1,
        head_j2,
        left_leg_j2,
        left_limb_j2,
        right_leg_j2,
        right_limb_j2,
        left_leg_j3,
        left_limb_j3,
        right_leg_j3,
        right_limb_j3,
        left_limb_j4,
        right_limb_j4,
        left_leg_j4,
        left_limb_j5,
        right_leg_j4,
        right_limb_j5,
        left_leg_j5,
        left_limb_j6,
        right_leg_j5,
        right_limb_j6,
        left_leg_j6,
        left_limb_j7,
        right_leg_j6,
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
        right_thumb_j2
    ]
    # endregion

# region 仿真环境取值赋值区域


def update_head_data(data):
    # /walker/head/joint_states
    set_joints[0] = data.position[0]
    set_joints[5] = data.position[1]
    # 计数器，计数关节更新次数
    global counter
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_Leg_data(data):
    # /walker/Leg/MeasuredJoint Webots中的腿没有启动
    global body_to_lhipyaw, lhipyaw_to_lhiproll, lhiproll_to_lhippitch, lhippitch_to_lkneepitch, lkneepitch_to_lanklepitch, lanklepitch_to_lankleroll # noqa
    global body_to_rhipyaw, rhipyaw_to_rhiproll, rhiproll_to_rhippitch, rhippitch_to_rkneepitch, rkneepitch_to_ranklepitch, ranklepitch_to_rankleroll # noqa
    leg_j.position = data.position
    leg_j.velocity = data.velocity
    leg_j.effort = data.effort
    body_to_lhipyaw = leg_j.position[0]
    lhipyaw_to_lhiproll = leg_j.position[1]
    lhiproll_to_lhippitch = leg_j.position[2]
    lhippitch_to_lkneepitch = leg_j.position[3]
    lkneepitch_to_lanklepitch = leg_j.position[4]
    lanklepitch_to_lankleroll = leg_j.position[5]
    body_to_rhipyaw = leg_j.position[6]
    rhipyaw_to_rhiproll = leg_j.position[7]
    rhiproll_to_rhippitch = leg_j.position[8]
    rhippitch_to_rkneepitch = leg_j.position[9]
    rkneepitch_to_ranklepitch = leg_j.position[10]
    ranklepitch_to_rankleroll = leg_j.position[11]
    set_joints[1] = body_to_lhipyaw
    set_joints[3] = body_to_rhipyaw
    set_joints[6] = lhipyaw_to_lhiproll
    set_joints[8] = rhipyaw_to_rhiproll
    set_joints[10] = lhiproll_to_lhippitch
    set_joints[12] = rhiproll_to_rhippitch
    set_joints[16] = lhippitch_to_lkneepitch
    set_joints[18] = rhippitch_to_rkneepitch
    set_joints[20] = lkneepitch_to_lanklepitch
    set_joints[22] = rkneepitch_to_ranklepitch
    set_joints[24] = lanklepitch_to_lankleroll
    set_joints[26] = ranklepitch_to_rankleroll
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_leftLimb_data(data):
    # /walker/leftLimb/joint_states
    set_joints[2] = data.position[0]
    set_joints[7] = data.position[1]
    set_joints[11] = data.position[2]
    set_joints[14] = data.position[3]
    set_joints[17] = data.position[4]
    set_joints[21] = data.position[5]
    set_joints[25] = data.position[6]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_rightLimb_data(data):
    # /walker/rightLimb/joint_states
    set_joints[4] = data.position[0]
    set_joints[9] = data.position[1]
    set_joints[13] = data.position[2]
    set_joints[15] = data.position[3]
    set_joints[19] = data.position[4]
    set_joints[23] = data.position[5]
    set_joints[27] = data.position[6]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_leftHand_data(data):
    # /walker/leftHand/joint_states
    #     name:
    #   - left_thumb
    #   - left_index
    #   - left_middle
    #   - left_ring
    #   - left_little
    global LFirstFinger1, LFirstFinger2
    global LSecondFinger1, LSecondFinger2, LThirdFinger1, LThirdFinger2, LForthFinger1, LForthFinger2, LFifthFinger1, LFifthFinger2 # noqa
    left_hand.position = data.position
    left_hand.velocity = data.velocity
    left_hand.effort = data.effort
    LFirstFinger1 = left_hand.position[0]
    LFirstFinger2 = LFirstFinger1
    LSecondFinger1 = left_hand.position[1]
    LSecondFinger2 = LSecondFinger1
    LThirdFinger1 = left_hand.position[2]
    LThirdFinger2 = LThirdFinger1
    LForthFinger1 = left_hand.position[3]
    LForthFinger2 = LForthFinger1
    LFifthFinger1 = left_hand.position[4]
    LFifthFinger2 = LFifthFinger1
    set_joints[32] = LFirstFinger1
    set_joints[42] = LFirstFinger2
    set_joints[28] = LSecondFinger1
    set_joints[38] = LSecondFinger2
    set_joints[29] = LThirdFinger1
    set_joints[39] = LThirdFinger2
    set_joints[31] = LForthFinger1
    set_joints[41] = LForthFinger2
    set_joints[30] = LFifthFinger1
    set_joints[40] = LFifthFinger2
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_rightHand_data(data):
    # /walker/rightHand/joint_states
    global RFirstFinger1, RFirstFinger2
    global RSecondFinger1, RSecondFinger2, RThirdFinger1, RThirdFinger2, RForthFinger1, RForthFinger2, RFifthFinger1, RFifthFinger2 # noqa
    right_hand.position = data.position
    right_hand.velocity = data.velocity
    right_hand.effort = data.effort
    RFirstFinger1 = right_hand.position[0]
    RFirstFinger2 = RFirstFinger1
    RSecondFinger1 = right_hand.position[1]
    RSecondFinger2 = RSecondFinger1
    RThirdFinger1 = right_hand.position[2]
    RThirdFinger2 = RThirdFinger1
    RForthFinger1 = right_hand.position[3]
    RForthFinger2 = RForthFinger1
    RFifthFinger1 = right_hand.position[4]
    RFifthFinger2 = RFifthFinger1
    set_joints[37] = RFirstFinger1
    set_joints[47] = RFirstFinger2
    set_joints[33] = RSecondFinger1
    set_joints[43] = RSecondFinger2
    set_joints[34] = RThirdFinger1
    set_joints[44] = RThirdFinger2
    set_joints[36] = RForthFinger1
    set_joints[46] = RForthFinger2
    set_joints[35] = RFifthFinger1
    set_joints[45] = RFifthFinger2
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()
# endregion


def update_realHead_data(data):
    # /walker/head/joint_states，
    # head_j1 head_j2
    set_joints[0] = data.position[0]
    set_joints[5] = data.position[1]
    # 计数器，计数关节更新次数
    global counter
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_realLeg_data(data):
    # SERVO_LHipYaw, SERVO_LHipRoll, SERVO_LHipPitch, SERVO_LKneePitch, SERVO_LAnklePitch, SERVO_LAnkleRoll, # noqa
    # SERVO_RHipYaw, SERVO_RHipRoll, SERVO_RHipPitch, SERVO_RKneePitch, SERVO_RAnklePitch, SERVO_RAnkleRoll, # noqa
    # 左腿 1 6 10 16 20 24
    set_joints[1] = data.position[0]
    set_joints[6] = data.position[1]
    set_joints[10] = data.position[2]
    set_joints[16] = data.position[3]
    set_joints[20] = data.position[4]
    set_joints[24] = data.position[5]
    # 右腿 3 8 12 18 22 26
    set_joints[3] = data.position[6]
    set_joints[8] = data.position[7]
    set_joints[12] = data.position[8]
    set_joints[18] = data.position[9]
    set_joints[22] = data.position[10]
    set_joints[26] = data.position[11]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_reallLimb_data(data):
    # 左臂 2 7 11 14 17 21 25
    set_joints[2] = data.position[0]
    set_joints[7] = data.position[1]
    set_joints[11] = data.position[2]
    set_joints[14] = data.position[3]
    set_joints[17] = data.position[4]
    set_joints[21] = data.position[5]
    set_joints[25] = data.position[6]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_realrLimb_data(data):
    # 右臂 4 9 13 15 19 23 27
    set_joints[4] = data.position[0]
    set_joints[9] = data.position[1]
    set_joints[13] = data.position[2]
    set_joints[15] = data.position[3]
    set_joints[19] = data.position[4]
    set_joints[23] = data.position[5]
    set_joints[27] = data.position[6]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_real_lHand_data(data):
    # /walker/leftHand/joint_states
    #     name:
    #   - left_thumb
    #   - left_index
    #   - left_middle
    #   - left_ring
    # 合拢时四指的set_joint[i]的值是1对应发来的状态是-1.97，张开是0对应0
    # deg2rad_ratio = 1.2/100  # 角度转弧度的比例系数
    vice_ratio = -0.5  # [-2, 0]->[1, 0]的映射系数
    set_joints[32] = data.position[0]*vice_ratio/6 # 第一指第n指节 # noqa
    set_joints[42] = set_joints[32]
    set_joints[28] = data.position[1]*vice_ratio
    set_joints[38] = set_joints[28]
    set_joints[29] = data.position[2]*vice_ratio
    set_joints[39] = set_joints[29]
    set_joints[31] = data.position[3]*vice_ratio
    set_joints[41] = set_joints[31]
    set_joints[30] = set_joints[31]  # 第五指
    set_joints[40] = set_joints[30]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_real_rHand_data(data):
    # /walker/rightHand/joint_states
    #     name:
    #   - right_thumb
    #   - right_index
    #   - right_middle
    #   - right_ring
    # 手抓是[-2.2,-2.2,-2.2,-2.2]，开是[-0.5,0,0,0]
    vice_ratio = -0.5  # [-2.2, 0]->[1, 0]的映射系数
    # vice_ratio = 1/(-2.2)
    set_joints[37] = data.position[0]*vice_ratio/6 # 第一指第n指节 # noqa
    set_joints[47] = set_joints[37]
    set_joints[33] = data.position[1]*vice_ratio
    set_joints[43] = set_joints[33]
    set_joints[34] = data.position[2]*vice_ratio
    set_joints[44] = set_joints[34]
    set_joints[36] = data.position[3]*vice_ratio
    set_joints[46] = set_joints[36]
    set_joints[35] = set_joints[36]  # 第五指
    set_joints[45] = set_joints[35]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def publishJoint():
    joint_state = JointState()
    joint_state.name = [
        "head_j1",  # 0
        "left_leg_j1",  # 1
        "left_limb_j1",  # 2
        "right_leg_j1",  # 3
        "right_limb_j1",
        "head_j2",  # 5
        "left_leg_j2",  # 6
        "left_limb_j2",  # 7
        "right_leg_j2",  # 8
        "right_limb_j2",
        "left_leg_j3",  # [10]
        "left_limb_j3",  # 11
        "right_leg_j3",  # 12
        "right_limb_j3",
        "left_limb_j4",  # 14
        "right_limb_j4",
        "left_leg_j4",  # 16
        "left_limb_j5",  # 17
        "right_leg_j4",  # 18
        "right_limb_j5",
        "left_leg_j5",  # [20]
        "left_limb_j6",  # 21
        "right_leg_j5",  # 22
        "right_limb_j6",
        "left_leg_j6",  # 24
        "left_limb_j7",  # 25
        "right_leg_j6",  # 26
        "right_limb_j7",  # 27
        "left_index_j1",  # 28
        "left_middle_j1",  # 29
        "left_pinky_j1",  # [30]
        "left_ring_j1",  # 31
        "left_thumb_j1",  # 32
        "right_index_j1",
        "right_middle_j1",
        "right_pinky_j1",
        "right_ring_j1",
        "right_thumb_j1",
        "left_index_j2",  # 38
        "left_middle_j2",  # 39
        "left_pinky_j2",  # [40]
        "left_ring_j2",  # 41
        "left_thumb_j2",  # 42
        "right_index_j2",
        "right_middle_j2",
        "right_pinky_j2",
        "right_ring_j2",
        "right_thumb_j2"  # 47
    ]
    num_joints = len(joint_state.name)
    joint_state.position = np.array([0.0] * num_joints)
    global set_joints
    joint_state.position = set_joints
    # 发布
    pub.publish(joint_state)
    global counter
    counter = 0


def transit():
    rospy.init_node('joint_state_transmitter', anonymous=True)
    global pub
    pub = rospy.Publisher('walker2_manual/joint_command', JointState, queue_size=10) # noqa
    real_flag = True
    if real_flag:
        # 在walker2真机上--腿、手掌、头和仿真环境下walkerX不同
        #rospy.Subscriber('/Leg/MeasuredJoint', JointState, update_realLeg_data) # noqa
        rospy.Subscriber('/ik_api_node/left_ikfk_pose', JointState, update_reallLimb_data) # noqa
        rospy.Subscriber('/ik_api_node/right_ikfk_pose', JointState, update_realrLimb_data) # noqa
        #rospy.Subscriber('/walker/leftHand/joint_states', JointState, update_real_lHand_data) # noqa
        #rospy.Subscriber('/walker/rightHand/joint_states', JointState, update_real_rHand_data) # noqa
        #rospy.Subscriber('/walker/head/joint_states', JointState, update_realHead_data) # 头的话题不对应，先不开 # noqa
    else:
        # 在webots中--
        # rospy.Subscriber('/Leg/MeasuredJoint', JointState, update_Leg_data) # noqa
        rospy.Subscriber('/ik_api_node/left_ikfk_pose', JointState, update_reallLimb_data) # noqa
        rospy.Subscriber('/ik_api_node/right_ikfk_pose', JointState, update_realrLimb_data) # noqa
        # rospy.Subscriber('/walker/leftHand/joint_states', JointState, update_leftHand_data) # noqa
        # rospy.Subscriber('/walker/rightHand/joint_states', JointState, update_rightHand_data) # noqa
        # rospy.Subscriber('/walker/head/joint_states', JointState, update_head_data) # noqa
    rospy.spin()  # 会停留在这里 一直订阅leftLimb_joint_states话题的内容


if __name__ == '__main__':
    initializeParams()
    transit()
