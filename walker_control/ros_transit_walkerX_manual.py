#!/usr/bin/env python

# 时间：20231030
# 功能：创建一个节点，订阅一个话题，在回调函数中 对接收到的/walker/leftLimb/joint_states话题数据进行处理，
# 然后对数据进行处理转发发布到另一个话题/joint_command。
# 1026新增：加入右臂话题的内容
# 1030新增：加入所有其他话题
# 1101新增：和walker2真机连接，其中有部分话题和webots不一样，针对性地修改
# 1114 手动控制

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
    right_thumb_j1 = 100.0
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


def update_head_data(data):
    # /walker/head/joint_states，简化版本
    # data.position[0]、[1]、[2]——name:- head_yaw- head_left- head_right
    # ——head_yaw、head_roll、head_pitch，对应关系不一定对，前两个是对应的，后面一个不对应
    set_joints[12] = data.position[0]
    set_joints[7] = data.position[1]
    set_joints[2] = data.position[2]
    # 计数器，计数关节更新次数
    global counter
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_Leg_data(data):
    # 腿关节name为空，先参考walker2的WAIC文件，要在另外一个文件中启动
    # /walker/Leg/MeasuredJoint
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
    set_joints[0] = body_to_lhipyaw
    set_joints[1] = body_to_rhipyaw
    set_joints[5] = lhipyaw_to_lhiproll
    set_joints[6] = rhipyaw_to_rhiproll
    set_joints[10] = lhiproll_to_lhippitch
    set_joints[11] = rhiproll_to_rhippitch
    set_joints[15] = lhippitch_to_lkneepitch
    set_joints[16] = rhippitch_to_rkneepitch
    set_joints[19] = lkneepitch_to_lanklepitch
    set_joints[20] = rkneepitch_to_ranklepitch
    set_joints[23] = lanklepitch_to_lankleroll
    set_joints[24] = ranklepitch_to_rankleroll
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_leftLimb_data(data):
    # /walker/leftLimb/joint_states
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
    # /walker/rightLimb/joint_states
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


def update_leftHand_data(data):
    # /walker/leftHand/joint_states
    #     name:
    #   - left_thumb1
    #   - left_thumb2
    #   - left_index
    #   - left_middle
    #   - left_ring
    #   - left_little
    # 手的关节不对应，看看用比例驱动第二个关节能不能行
    global LFirstFinger1, LFirstFinger2, LFirstFinger3, LFirstFinger4
    global LSecondFinger1, LSecondFinger2, LThirdFinger1, LThirdFinger2, LForthFinger1, LForthFinger2, LFifthFinger1, LFifthFinger2 # noqa
    left_hand.position = data.position
    left_hand.velocity = data.velocity
    left_hand.effort = data.effort
    LFirstFinger1 = left_hand.position[0]
    LFirstFinger2 = left_hand.position[0]*0.5
    LFirstFinger3 = left_hand.position[1]
    LFirstFinger4 = left_hand.position[1]*0.5
    LSecondFinger1 = left_hand.position[2]
    LSecondFinger2 = left_hand.position[2]*0.5
    LThirdFinger1 = left_hand.position[3]
    LThirdFinger2 = left_hand.position[3]*0.5
    LForthFinger1 = left_hand.position[4]
    LForthFinger2 = left_hand.position[4]*0.5
    LFifthFinger1 = left_hand.position[5]
    LFifthFinger2 = left_hand.position[5]*0.5
    set_joints[33] = LFirstFinger1
    set_joints[43] = LFirstFinger2
    set_joints[49] = LFirstFinger3
    set_joints[51] = LFirstFinger4
    set_joints[29] = LSecondFinger1
    set_joints[39] = LSecondFinger2
    set_joints[30] = LThirdFinger1
    set_joints[40] = LThirdFinger2
    set_joints[32] = LForthFinger1
    set_joints[42] = LForthFinger2
    set_joints[31] = LFifthFinger1
    set_joints[41] = LFifthFinger2
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_rightHand_data(data):
    # /walker/rightHand/joint_states
    global RFirstFinger1, RFirstFinger2, RFirstFinger3, RFirstFinger4
    global RSecondFinger1, RSecondFinger2, RThirdFinger1, RThirdFinger2, RForthFinger1, RForthFinger2, RFifthFinger1, RFifthFinger2 # noqa
    right_hand.position = data.position
    right_hand.velocity = data.velocity
    right_hand.effort = data.effort
    RFirstFinger1 = right_hand.position[0]
    RFirstFinger2 = right_hand.position[0]*0.5
    RFirstFinger3 = right_hand.position[1]
    RFirstFinger4 = right_hand.position[1]*0.5
    RSecondFinger1 = right_hand.position[2]
    RSecondFinger2 = right_hand.position[2]*0.5
    RThirdFinger1 = right_hand.position[3]
    RThirdFinger2 = right_hand.position[3]*0.5
    RForthFinger1 = right_hand.position[4]
    RForthFinger2 = right_hand.position[4]*0.5
    RFifthFinger1 = right_hand.position[5]
    RFifthFinger2 = right_hand.position[5]*0.5
    set_joints[38] = RFirstFinger1
    set_joints[48] = RFirstFinger2
    set_joints[50] = RFirstFinger3
    set_joints[52] = RFirstFinger4
    set_joints[34] = RSecondFinger1
    set_joints[44] = RSecondFinger2
    set_joints[35] = RThirdFinger1
    set_joints[45] = RThirdFinger2
    set_joints[37] = RForthFinger1
    set_joints[47] = RForthFinger2
    set_joints[36] = RFifthFinger1
    set_joints[46] = RFifthFinger2
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_realHead_data(data):
    # /walker/head/joint_states，
    # head_j1 head_j2
    set_joints[12] = data.position[0]
    set_joints[7] = data.position[1]
    set_joints[2] = 0.0
    # 计数器，计数关节更新次数
    global counter
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_realLeg_data(data):
    # SERVO_LHipYaw, SERVO_LHipRoll, SERVO_LHipPitch, SERVO_LKneePitch, SERVO_LAnklePitch, SERVO_LAnkleRoll, # noqa
    # SERVO_RHipYaw, SERVO_RHipRoll, SERVO_RHipPitch, SERVO_RKneePitch, SERVO_RAnklePitch, SERVO_RAnkleRoll, # noqa
    set_joints[0] = data.position[0]
    set_joints[1] = data.position[6]
    set_joints[5] = data.position[1]
    set_joints[6] = data.position[7]
    set_joints[10] = data.position[2]
    set_joints[11] = data.position[8]
    set_joints[15] = data.position[3]
    set_joints[16] = data.position[9]
    set_joints[19] = data.position[4]
    set_joints[20] = data.position[10]
    set_joints[23] = data.position[5]
    set_joints[24] = data.position[11]
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def update_real_lHand_data(data):
    # /walker/leftHand/joint_states ?
    #     name:
    #   - left_thumb
    #   - left_index
    #   - left_middle
    #   - left_ring
    # 合拢时四指的set_joint[i]的值是1对应发来的状态是-1.97，张开是0对应0
    # deg2rad_ratio = 1.2/100  # 角度转弧度的比例系数
    vice_ratio = -0.5  # [-2, 0]->[1, 0]的映射系数
    set_joints[33] = 0.0
    set_joints[43] = set_joints[33]
    set_joints[49] = data.position[0]*vice_ratio # 第一指第n指节 # noqa
    set_joints[51] = set_joints[39]
    set_joints[29] = data.position[1]*vice_ratio
    set_joints[39] = set_joints[29]
    set_joints[30] = data.position[2]*vice_ratio
    set_joints[40] = set_joints[30]
    set_joints[32] = data.position[3]*vice_ratio
    set_joints[42] = set_joints[32]
    set_joints[31] = set_joints[32]  # 第五指
    set_joints[41] = set_joints[32]
    # fixed_ratio = 1.2/100
    # set_joints[33] = max(0, min(100, data.position[0]*fixed_ratio))
    # set_joints[43] = set_joints[33]
    # set_joints[49] = 0.0  # 第一指第n指节
    # set_joints[51] = 0.0
    # set_joints[29] = max(0, min(100, data.position[1]*fixed_ratio))
    # set_joints[39] = set_joints[29]
    # set_joints[30] = max(0, min(100, data.position[2]*fixed_ratio))
    # set_joints[40] = set_joints[30]
    # set_joints[32] = max(0, min(100, data.position[3]*fixed_ratio))
    # set_joints[42] = set_joints[32]
    # set_joints[31] = 0.0  # 第五指
    # set_joints[41] = 0.0
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
    # 手抓是[-2,-2,-2,-2]，开是[-0.5,0,0,0]
    vice_ratio = -0.5  # [-2, 0]->[1, 0]的映射系数
    set_joints[38] = 0.0
    set_joints[48] = set_joints[38]
    set_joints[50] = -data.position[0]*vice_ratio # 第一指第n指节 # noqa
    set_joints[52] = set_joints[50]
    set_joints[34] = data.position[1]*vice_ratio
    set_joints[44] = set_joints[34]
    set_joints[35] = data.position[2]*vice_ratio
    set_joints[45] = set_joints[35]
    set_joints[37] = data.position[3]*vice_ratio
    set_joints[47] = set_joints[37]
    set_joints[36] = set_joints[37]  # 第五指
    set_joints[46] = set_joints[37]
    # fixed_ratio = 1.2/100
    # set_joints[38] = max(0, min(100, data.position[0]*fixed_ratio))
    # set_joints[48] = max(0, min(100, data.position[0]*fixed_ratio))
    # set_joints[50] = 0.0  # 第一指第n指节
    # set_joints[52] = 0.0
    # set_joints[34] = max(0, min(100, data.position[1]*fixed_ratio))
    # set_joints[44] = max(0, min(100, data.position[1]*fixed_ratio))
    # set_joints[35] = max(0, min(100, data.position[2]*fixed_ratio))
    # set_joints[45] = max(0, min(100, data.position[2]*fixed_ratio))
    # set_joints[37] = max(0, min(100, data.position[3]*fixed_ratio))
    # set_joints[47] = max(0, min(100, data.position[3]*fixed_ratio))
    # set_joints[36] = 0.0  # 第五指
    # set_joints[46] = 0.0
    # 计数器，计数关节更新次数
    global counter, joint_num
    counter += 1
    if counter == joint_num:
        publishJoint()


def publishJoint():
    joint_state = JointState()
    joint_state.name = [
        "body_to_lhipyaw",  # [0]
        "body_to_rhipyaw",
        "headpitch",
        "left_limb_j1",
        "right_limb_j1",
        "lhipyaw_to_lhiproll",
        "rhipyaw_to_rhiproll",
        "headroll",
        "left_limb_j2",
        "right_limb_j2",
        "lhiproll_to_lhippitch",  # [10]
        "rhiproll_to_rhippitch",
        "headyaw",
        "left_limb_j3",
        "right_limb_j3",
        "lhippitch_to_lkneepitch",
        "rhippitch_to_rkneepitch",
        "left_limb_j4",
        "right_limb_j4",
        "lkneepitch_to_lanklepitch",  # noqa
        "rkneepitch_to_ranklepitch",  # [20]
        "left_limb_j5",
        "right_limb_j5",
        "lanklepitch_to_lankleroll",
        "ranklepitch_to_rankleroll",
        "left_limb_j6",
        "right_limb_j6",
        "left_limb_j7",
        "right_limb_j7",
        "left_index_j1",
        "left_middle_j1",  # [30]
        "left_pinky_j1",
        "left_ring_j1",
        "left_thumb_j1",
        "right_index_j1",
        "right_middle_j1",
        "right_pinky_j1",
        "right_ring_j1",
        "right_thumb_j1",
        "left_index_j2",
        "left_middle_j2",  # [40]
        "left_pinky_j2",
        "left_ring_j2",
        "left_thumb_j2",
        "right_index_j2",
        "right_middle_j2",
        "right_pinky_j2",
        "right_ring_j2",
        "right_thumb_j2",
        "left_thumb_j3",
        "right_thumb_j3",  # [50]
        "left_thumb_j4",
        "right_thumb_j4",
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
    pub = rospy.Publisher('walkerX_manual/joint_command', JointState, queue_size=10) # noqa
    real_flag = False
    if real_flag:
        # 在walker2真机上，腿、手掌、头和仿真环境下walkerX不同
        rospy.Subscriber('/Leg/MeasuredJoint', JointState, update_realLeg_data) # noqa
        rospy.Subscriber('/walker/leftLimb/joint_states', JointState, update_leftLimb_data) # noqa
        rospy.Subscriber('/walker/rightLimb/joint_states', JointState, update_rightLimb_data) # noqa
        rospy.Subscriber('/walker/leftHand/joint_states', JointState, update_real_lHand_data) # noqa
        rospy.Subscriber('/walker/rightHand/joint_states', JointState, update_real_rHand_data) # noqa
        # rospy.Subscriber('/walker/head/joint_states', JointState, update_realHead_data) # 头的话题不对应，先不开 # noqa
    else:
        # 在webots中
        #rospy.Subscriber('/walker/Leg/MeasuredJoint', JointState, update_Leg_data) # noqa
        rospy.Subscriber('/ik_api_node/left_ikfk_pose', JointState, update_leftLimb_data) # noqa
        rospy.Subscriber('/ik_api_node/right_ikfk_pose', JointState, update_rightLimb_data) # noqa
        #rospy.Subscriber('/walker/leftHand/joint_states', JointState, update_leftHand_data) # noqa
        #rospy.Subscriber('/walker/rightHand/joint_states', JointState, update_rightHand_data) # noqa
        #rospy.Subscriber('/walker/head/joint_states', JointState, update_head_data) # noqa
    rospy.spin()  # 会停留在这里 一直订阅leftLimb_joint_states话题的内容


if __name__ == '__main__':
    initializeParams()
    transit()
