#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray


def main():

    rospy.init_node('relay_mover')

    subname = rospy.get_param('~relay_1_sub_name', default='relay_1_sub_')
    rospy.loginfo(subname)

    pub = rospy.Publisher(subname, Int16MultiArray, queue_size = 10)

    r = rospy.Rate(1)
    data_to_send = Int16MultiArray()
    array = [1, 1, 1, 1, 1, 1, 1, 1]
    while not rospy.is_shutdown():
        for i in range(0, 8):
            
            array[i] = 0
            rospy.loginfo("relay test loop {}".format(str(array)))
            data_to_send.data = array
            pub.publish(data_to_send)
            r.sleep();
        array = [1, 1, 1, 1, 1, 1, 1, 1]
        r.sleep();

if __name__ == '__main__':
    main()
