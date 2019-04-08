import rospy
from dbw_mkz_msgs.msg import ThrottleCmd

def talker():
    pub = rospy.Publisher('/carla/ThrottleCmd', ThrottleCmd, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    cmd = ThrottleCmd()
    cmd.pedal_cmd = 0.7
    cmd.pedal_cmd_type = 2

    while not rospy.is_shutdown():
        rospy.loginfo(cmd)
        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass