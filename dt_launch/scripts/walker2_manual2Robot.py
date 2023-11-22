#!/usr/bin/env python

# 时间：1115
# 功能：读取绿色机器人的状态发送给真机/webots端控制器

import rospy
from sensor_msgs.msg import JointState
from baxter_core_msgs.msg import JointCommand
import numpy as np


def initializeParams():
    global left_limb_j, right_limb_j
    left_limb_j = JointCommand()
    left_limb_j.mode = 5
    left_limb_j.names = ["llj1", "llj2", "llj3", "llj4", "llj5", "llj6", "llj7"] # noqa
    left_limb_j.command = np.array([0.0]*len(left_limb_j.names))
    right_limb_j = JointCommand()
    left_limb_j.mode = 5
    right_limb_j.names = ["rlj1", "rlj2", "rlj3", "rlj4", "rlj5", "rlj6", "rlj7"] # noqa
    right_limb_j.command = np.array([0.0]*len(right_limb_j.names))
    # left_limb_j = JointState()
    # left_limb_j.name = ["llj1", "llj2", "llj3", "llj4", "llj5", "llj6", "llj7"] # noqa
    # left_limb_j.position = np.array([0.0]*len(left_limb_j.name))
    # right_limb_j = JointState()
    # right_limb_j.name = ["rlj1", "rlj2", "rlj3", "rlj4", "rlj5", "rlj6", "rlj7"] # noqa
    # right_limb_j.position = np.array([0.0]*len(right_limb_j.name))


def update_limbs_data(data):
    # 左臂 2 7 11 14 17 21 25
    left_limb_j.command[0] = data.position[2]
    left_limb_j.command[1] = data.position[7]
    left_limb_j.command[2] = data.position[11]
    left_limb_j.command[3] = data.position[14]
    left_limb_j.command[4] = data.position[17]
    left_limb_j.command[5] = data.position[21]
    left_limb_j.command[6] = data.position[25]
    # 右臂 4 9 13 15 19 23 27
    right_limb_j.command[0] = data.position[4]
    right_limb_j.command[1] = data.position[9]
    right_limb_j.command[2] = data.position[13]
    right_limb_j.command[3] = data.position[15]
    right_limb_j.command[4] = data.position[19]
    right_limb_j.command[5] = data.position[23]
    right_limb_j.command[6] = data.position[27]
    publishJoint()


def publishJoint():
    pubLeft.publish(left_limb_j)
    pubRight.publish(right_limb_j)


def transit():
    rospy.init_node('joint_state_transmitter_Manual2Robot', anonymous=True)
    global pubLeft, pubRight
    pubLeft = rospy.Publisher('/walker/leftLimb/controller', JointCommand, queue_size=10) # noqa
    pubRight = rospy.Publisher('/walker/rightLimb/controller', JointCommand, queue_size=10) # noqa
    rospy.Subscriber('walker2_manual/joint_states', JointState, update_limbs_data) # noqa
    rospy.spin()  # 会停留在这里 一直订阅leftLimb_joint_states话题的内容


if __name__ == '__main__':
    initializeParams()
    transit()
