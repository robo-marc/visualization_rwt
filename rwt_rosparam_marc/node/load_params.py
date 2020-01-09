#!/usr/bin/env python

# import roslib; roslib.load_manifest('rwt_rosparam_marc')
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
        # if ns in doc:
        #     ns = ns_join(default_namespace, doc.get(NS, None))
        #     if verbose:
        #         print("reading parameters into namespace [%s]"%ns)
        #     del doc[NS]
        # else:
            # ns = default_namespace
        paramList.append((doc, ns))

    for params,ns in paramList:
        rosparam.upload_params(ns, params)

    # yaml_load = ast.literal_eval(req.params)
    # rosparam.upload_params(ns, yaml_load)


    # return 
    tree = rosparam.get_param(script_resolve_name('rosparam', ns))
    # paramList.append(str(tree))

    dump = yaml.dump(tree)
    if dump.endswith('\n...\n'):
        dump = dump[:-5]

    # paramList.append(str("%s\n"%(dump)))    
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

    # TODO FIXME test load file2
    # ns = ''
    # yaml_load = robot_state_publisher: {publish_frequency: 5.0}
    # rosparam.upload_params(ns, yaml_load)

    # TODO test 

    rospy.spin()

if __name__ == "__main__":
    load_params()
