#!/usr/bin/env python

import roslib; roslib.load_manifest('rwt_rosparam_marc')
import sys
import rosmsg
import rospy
from rospkg.rospack import RosPack
from rwt_rosparam_marc.srv import *
import rosparam
from optparse import OptionParser
import yaml
import rosgraph
from rosgraph.names import script_resolve_name, ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS

# def dump_params(argv):

def handle_rwt_rosparam_marc(req):
    ns = ''
    tree = rosparam.get_param(script_resolve_name('rosparam',ns))
    f = sys.stdout
    # yaml.dump(tree, f)
    # f = open(filename, 'w')
    # try:
    #     yaml.dump(tree, f)
    # finally:
    #     f.close()
    paramList = yaml.dump(tree, f)
    # paramList.append(str(ns))
    # paramList.append(str(f))
    print (f)
    print (tree)
    print (ns)
    return DumpParamsResponse(paramList)

def dump_params():
    rospy.init_node('dump_params')
    s = rospy.Service('dump_params', DumpParams, handle_rwt_rosparam_marc)
    print ("stanby ok.")
    rospy.spin()

if __name__ == "__main__":
    dump_params()
