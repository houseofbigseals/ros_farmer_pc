#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray


def main():

    rospy.init_node('relay_mover_2')

    subname = rospy.get_param('~relay_1_sub_name', default='relay_1_sub_')
    rospy.loginfo(subname)

    pub = rospy.Publisher(subname, Int16MultiArray, queue_size = 10)

    r = rospy.Rate(1)
    data_to_send = Int16MultiArray()
    array = [1, 1, 1, 1, 1, 1, 1, 1]
    while not rospy.is_shutdown():

        ch = int(input('please input channel from 0 to 7'))
        if ch > 7 or ch <0:
            ospy.logerror('one relay test: incorrect data to set ralay')
        else:
            value = int(input('input 0 to set on, input 1 to set off'))
            # add new value to array
            array[ch] = value
            rospy.loginfo("one relay test: set new state  {}".format(str(array)))

            data_to_send.data = array
            pub.publish(data_to_send)
            r.sleep();

if __name__ == '__main__':
    main()
