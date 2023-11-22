# region 同时订阅和发布话题

# import rospy
# from sensor_msgs.msg import JointState


# def callback(data):
#     pub.publish(data)


# rospy.init_node('joint_state_transmitter', anonymous=True)
# pub = rospy.Publisher('llj_joint_command', JointState, queue_size=10)
# rospy.Subscriber('joint_states', JointState, callback)
# rospy.spin()

# endregion

# region 如果我想要把三个订阅的话题内容处理成一个数据，再把它进行发布，如何用代码实现
import rospy
from sensor_msgs.msg import JointState


def callback1(data):
    # 处理第一个话题的数据
    global a
    a += data
    process_data()


def callback2(data):
    # 处理第二个话题的数据
    global a
    a += data
    process_data()


def callback3(data):
    global a
    a += data
    # 处理第三个话题的数据
    process_data()


def process_data():
    # 在这里进行数据处理的逻辑
    # 假设你想将三个话题的数据拼接成一个列表
    data_list = [data1, data2, data3]  # 这里的data1, data2, data3是你从每个回调函数中获取的数据 # noqa
    print(a)
    # 将数据列表发布到话题中
    pub.publish(data_list)


rospy.init_node('joint_state_transmitter', anonymous=True)
pub = rospy.Publisher('llj_joint_command', JointState, queue_size=10)

rospy.Subscriber('topic1', JointState, callback1)
rospy.Subscriber('topic2', JointState, callback2)
rospy.Subscriber('topic3', JointState, callback3)

rospy.spin()

# endregion

# region 储存话题数据
# import rospy
# from sensor_msgs.msg import JointState


# def callback(data):
#     # 在这里处理接收到的数据
#     # 将数据保存到txt文件中
#     with open('/home/walker2/noetic_ws/src/walker_control/Jointdata.txt', 'a') as file: # noqa
#         file.write(str(data) + '\n')


# def save_joint_states():
#     rospy.init_node('joint_state_subscriber', anonymous=True)
#     rospy.Subscriber('joint_states', JointState, callback)
#     rospy.Subscriber('joint_command', JointState, callback)
#     rospy.Subscriber('/walker/leftLimb/joint_states', JointState, callback)
#     rospy.spin()


# if __name__ == '__main__':
#     save_joint_states()

# endregion
