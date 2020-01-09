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
    # paramList.append(str(tree))

    # return DumpParamsResponse(paramList)

    # TODO test start
    dump = yaml.dump(tree)
    if dump.endswith('\n...\n'):
        dump = dump[:-5]
    # return DumpParamsResponse("%s\n"%(dump))
        
    # paramList.append(str("%s\n"%(dump)))    
    paramList.append("%s\n"%(dump))    
    return DumpParamsResponse(paramList)




def dump_params():
    rospy.init_node('dump_params')
    s = rospy.Service('dump_params', DumpParams, handle_rwt_rosparam_marc)
    
    # TODO test start
    paramList = []
    ns = ''
    tree = rosparam.get_param(script_resolve_name('rosparam', ns))

    paramList.append(str(tree))
    print('---- paramList(tree) ----')
    print(paramList)

    dump = yaml.dump(tree)
    if dump.endswith('\n...\n'):
        dump = dump[:-5]
    print('---- dump ----')    
    print("%s\n"%(dump))

    # TODO test end

    rospy.spin()

if __name__ == "__main__":
    dump_params()
