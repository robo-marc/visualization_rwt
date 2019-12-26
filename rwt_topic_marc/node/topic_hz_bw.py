#!/usr/bin/env python

from __future__ import division, with_statement
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import roslib
import rospy
from rostopic import ROSTopicHz
import roslib; roslib.load_manifest('rwt_topic_marc')
import sys
import rosmsg
import rospy
from rospkg.rospack import RosPack
import rosparam
from optparse import OptionParser
import yaml
import rosgraph
from rosgraph.names import script_resolve_name, ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS
from rwt_topic_marc.srv import *

# def topic_info(argv):

class TopicInfo(ROSTopicHz):

    def __init__(self, topic_name, topic_type):
        super(TopicInfo, self).__init__(100)
        self._topic_name = topic_name
        self.error = None
        self._subscriber = None
        self.monitoring = False
        self._reset_data()
        self.message_class = None
        try:
            self.message_class = roslib.message.get_message_class(topic_type)
        except Exception as e:
            self.message_class = None
            # qWarning('TopicInfo.__init__(): %s' % (e))

        if self.message_class is None:
            self.error = 'can not get message class for type "%s"' % topic_type
            # qWarning('TopicInfo.__init__(): topic "%s": %s' % (topic_name, self.error))

    def _reset_data(self):
        self.last_message = None
        self.times = []
        self.timestamps = []
        self.sizes = []

    def toggle_monitoring(self):
        if self.monitoring:
            self.stop_monitoring()
        else:
            self.start_monitoring()

    def start_monitoring(self):
        if self.message_class is not None:
            self.monitoring = True
            # FIXME: subscribing to class AnyMsg breaks other subscribers on same node
            self._subscriber = rospy.Subscriber(self._topic_name, self.message_class, self.message_callback)

    def stop_monitoring(self):
        self.monitoring = False
        self._reset_data()
        if self._subscriber is not None:
            self._subscriber.unregister()

    def message_callback(self, message):
        ROSTopicHz.callback_hz(self, message)
        with self.lock:
            self.timestamps.append(rospy.get_time())

            # FIXME: this only works for message of class AnyMsg
            #self.sizes.append(len(message._buff))
            # time consuming workaround...
            buff = StringIO()
            message.serialize(buff)
            self.sizes.append(len(buff.getvalue()))

            if len(self.timestamps) > self.window_size - 1:
                self.timestamps.pop(0)
                self.sizes.pop(0)
            assert(len(self.timestamps) == len(self.sizes))

            self.last_message = message

    def get_bw(self):
        if len(self.timestamps) < 2:
            return None, None, None, None
        current_time = rospy.get_time()
        if current_time <= self.timestamps[0]:
            return None, None, None, None
            
        with self.lock:
            total = sum(self.sizes)
            bytes_per_s = total / (current_time - self.timestamps[0])
            mean_size = total / len(self.timestamps)
            max_size = max(self.sizes)
            min_size = min(self.sizes)
            return bytes_per_s, mean_size, min_size, max_size

    def get_hz(self):
        if not self.times:
            return None, None, None, None
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1. / mean if mean > 0. else 0
            min_delta = min(self.times)
            max_delta = max(self.times)
        return rate, mean, min_delta, max_delta

def handle_rwt_rostopic_marc(req):
    topic = TopicInfo(req.name, req.type)
    rate, mean, min_delta, max_delta = topic.get_hz()
    bytes_per_s, mean_size, min_size, max_size = topic.get_bw()
    print(rate)
    print(bytes_per_s)
    return TopicHzBwResponse(rate, bytes_per_s)

def topic_hz_bw():
    rospy.init_node('topic_hz_bw')
    s = rospy.Service('topic_hz_bw', TopicHzBw, handle_rwt_rostopic_marc)
    print "Ready"
    rospy.spin()

if __name__ == "__main__":
    topic_hz_bw()