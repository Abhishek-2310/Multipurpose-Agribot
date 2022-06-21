import rospy
from std_msgs.msg import Float64
# from std_msgs.msg import String


# def callback(command):
#     rospy.loginfo(command.data)

rospy.init_node('motion', anonymous=True)
yaw_pub = rospy.Publisher(
    '/agribot_v2/yaw_position_controller/command', Float64, queue_size=10)
pitch_pub = rospy.Publisher(
    '/agribot_v2/pitch_position_controller/command', Float64, queue_size=10)

pitch_msg = -0.8
yaw_msg = 0.7

direction = -1


def yaw_func():
    global yaw_msg
    for num in range(0, 700):

        if yaw_msg <= -0.7:
            direction = 1

        if yaw_msg >= 0.7:
            direction = -1

        yaw_pub.publish(yaw_msg)
        rospy.loginfo(yaw_msg)
        yaw_msg = yaw_msg + (0.01 * direction)

        rate = rospy.Rate(50)  # 50hz
        rate.sleep()


def pitch_func():
    global pitch_msg
    for num in range(0, 700):

        if pitch_msg <= -0.8:
            direction = 1

        if pitch_msg >= 0.1:
            direction = -1

        pitch_pub.publish(pitch_msg)
        rospy.loginfo(pitch_msg)
        pitch_msg = pitch_msg + (0.01 * direction)

        rate = rospy.Rate(50)  # 50hz
        rate.sleep()


while not rospy.is_shutdown():

    num = int(input("Choose yaw (1) /pitch (2)"))

    if num == 1:
        yaw_func()

    if num == 2:
        pitch_func()

    # pitch_pub.publish(hello_str)


# if __name__ == '__main__':
#     try:
#         motion()
#     except rospy.ROSInterruptException:
#         pass
