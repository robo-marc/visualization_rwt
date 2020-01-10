#!/usr/bin/env python

import roslib; roslib.load_manifest('rwt_rosparam_marc')
import sys
import rosmsg
import rospy
from rwt_rosparam_marc.srv import *
import rosparam
import ast
import rosgraph
import yaml
from rosgraph.names import script_resolve_name, ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS


def handle_rwt_rosparam_marc_load(req):

    paramResponse = []
    paramList = []
    ns = ''

    for doc in yaml.load_all(req.params):
        paramList.append((doc, ns))

    for params,ns in paramList:
        rosparam.upload_params(ns, params)

    # TODO return test
    tree = rosparam.get_param(script_resolve_name('rosparam', ns))
    dump = yaml.dump(tree)
    if dump.endswith('\n...\n'):
        dump = dump[:-5]

    paramResponse.append("%s\n"%(dump))          

    return LoadParamsResponse(paramResponse) 


def load_params():
    rospy.init_node('load_params')
    s = rospy.Service('load_params', LoadParams, handle_rwt_rosparam_marc_load)

    # TODO FIXME test load file
    # testdict = {}
    # ns = ''
    # yaml_load = ast.literal_eval(req.params)
    # rosparam.upload_params(ns, yaml_load)
    # TODO test 

    rospy.spin()

if __name__ == "__main__":
    load_params()
