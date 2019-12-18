#!/usr/bin/env python

import roslib; roslib.load_manifest('rwt_srv_marc')
import rosmsg
import rospy
from rospkg.rospack import RosPack
from rwt_srv_marc.srv import *

def handle_rwt_srv(req):
    srv = rosmsg.MODE_SRV
    pkg = RosPack()
    package = rosmsg.iterate_packages(pkg,srv)
    messageList = []
    for pack in package:
        service,path = pack
        message = rosmsg.list_srvs(service)
        msgList = []
        for index, messageItem in enumerate(message):
            msgList.append(str(messageItem))
        messageList.append(str(message))
    return SrvListResponse(messageList)

def srv_list():
    rospy.init_node('srv_list')
    s = rospy.Service('srv_list', SrvList, handle_rwt_srv)
    print ("stunby ok.")
    rospy.spin()

if __name__ == "__main__":

    srv_list()
