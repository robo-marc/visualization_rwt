#!/usr/bin/env python

import rosmsg
import rospy
from rospkg.rospack import RosPack
from rwt_action_marc.srv import *
from rqt_py_common import rosaction

def handle_rwt_action(req):
    action = rosaction.MODE_ACTION
    pkg = RosPack()
    package = rosaction.iterate_packages(pkg,action)
    messageList = []
    for pack in package:
        action,path = pack
        message = rosmsg.list_msgs(action)
        messageList.append(str(message))
    return ActionListResponse(messageList)

def action_list():
    rospy.init_node('action_list')
    s = rospy.Service('action_list', ActionList, handle_rwt_action)
    rospy.spin()

if __name__ == "__main__":
    action_list()
