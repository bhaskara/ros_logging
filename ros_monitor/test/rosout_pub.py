#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')

import rospy

i = 0
names = ['foo', 'bar', 'baz']

rospy.init_node('rosout_pub')
while not rospy.is_shutdown():
    rospy.loginfo("{0} {1}".format(names[i%3], i))
    rospy.sleep(1.0)
    i += 1
