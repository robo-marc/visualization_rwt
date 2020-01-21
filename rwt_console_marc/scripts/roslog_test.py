#!/usr/bin/env python
import rospy

rospy.init_node('test', anonymous=True)

r = rospy.Rate(20) # 20Hz * 5 messages
i = 0
while not rospy.is_shutdown():
    i += 1
    rospy.logdebug('debug: {}'.format(i))
    rospy.loginfo('info: {}'.format(i))
    rospy.logwarn('warn: {}'.format(i))
    rospy.logerr('error: {}'.format(i))
    rospy.logfatal('fatal: {}'.format(i))
    r.sleep()
