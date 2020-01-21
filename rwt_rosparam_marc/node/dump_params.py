#!/usr/bin/env python

import roslib; roslib.load_manifest('rwt_rosparam_marc')
import sys
import rosmsg
import rospy
from rwt_rosparam_marc.srv import *
import rosparam
import rosgraph
import yaml
from rosgraph.names import script_resolve_name, ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS


def handle_rwt_rosparam_marc(req):

    paramList = []
    ns = ''
    tree = rosparam.get_param(script_resolve_name('rosparam', ns))
    dump = yaml.dump(tree)
    if dump.endswith('\n...\n'):
        dump = dump[:-5]
    paramList.append("%s\n" % (dump))
    
    return DumpParamsResponse(paramList)

def dump_params():
    rospy.init_node('dump_params')
    s = rospy.Service('dump_params', DumpParams, handle_rwt_rosparam_marc)

    rospy.spin()

if __name__ == "__main__":
    dump_params()
