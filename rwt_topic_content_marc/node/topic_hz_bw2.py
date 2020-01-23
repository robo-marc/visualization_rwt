#!/usr/bin/env python

from __future__ import division, with_statement
try:
    from cStringIO import StringIO
except ImportError:
    from io import StringIO
import roslib
import rospy
from rostopic import ROSTopicHz
import sys
import rosmsg
import rospy
from rospkg.rospack import RosPack
import rosparam
from optparse import OptionParser
import yaml
import time
import rosgraph
from rosgraph.names import script_resolve_name, ns_join, get_ros_namespace, make_caller_id, make_global_ns, GLOBALNS
from rwt_topic_content_marc.srv import StartMonitoring, StartMonitoringResponse, \
    StopMonitoring, StopMonitoringResponse, \
    GetMonitoringInfo, GetMonitoringInfoResponse
from rwt_topic_content_marc.msg import MonitoringInfo
from std_srvs.srv import Trigger, TriggerResponse

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
        if self.message_class is None:
            self.error = 'can not get message class for type "%s"' % topic_type

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
        print(' ---- start get_bw ---- ')
        # print(self.timestamps)
        if len(self.timestamps) < 2:
            return None, None, None, None
        current_time = rospy.get_time()
        # print(current_time)
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
        print(' ---- start get_hz ---- ')
        # print(self.times)
        if not self.times:
            return None, None, None, None
        with self.lock:
            n = len(self.times)
            mean = sum(self.times) / n
            rate = 1. / mean if mean > 0. else 0
            min_delta = min(self.times)
            max_delta = max(self.times)
        return rate, mean, min_delta, max_delta

class TopicService(object):
    def __init__(self):
        self._monitored_topics = {}
        rospy.Service('start_monitoring', StartMonitoring, self.handle_start_monitoring)
        rospy.Service('get_monitoring_info', GetMonitoringInfo, self.handle_get_monitoring_info)
        rospy.Service('stop_monitoring', StopMonitoring, self.handle_stop_monitoring)

    def handle_start_monitoring(self, req):
        topic = TopicInfo(req.name, req.type)
        topic.start_monitoring()
        key = (req.name, req.type)
        self._monitored_topics[key] = topic

        res = StartMonitoringResponse()
        return res

    def handle_get_monitoring_info(self, req):
        res = GetMonitoringInfoResponse()
        for key, info in self._monitored_topics.items(): 
            topic_name, topic_type = key
            rate, mean, min_delta, max_delta = info.get_hz()
            bytes_per_s, mean_size, min_size, max_size = info.get_bw()

            rate_text = '%1.2f' % rate if rate != None else 'unknown'

            if bytes_per_s is None:
                bandwidth_text = 'unknown'
            elif bytes_per_s < 1000:
                bandwidth_text = '%.2fB/s' % bytes_per_s
            elif bytes_per_s < 1000000:
                bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
            else:
                bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)    

            print(topic_name, topic_type, rate_text, bandwidth_text)
            topic_info = MonitoringInfo()
            topic_info.name = topic_name
            topic_info.type = topic_type
            topic_info.bw = bandwidth_text
            topic_info.hz = rate_text

            res.info.append(topic_info)

        return res

    def handle_stop_monitoring(self, req):
        key = (req.name, req.type)
        info = self._monitored_topics.pop(key, None)
        if info is not None:
            info.stop_monitoring()

        res = StopMonitoringResponse()
        return res

def topic_hz_bw2():
    rospy.init_node('topic_hz_bw2', anonymous=True)
    # rospy.init_node('topic_hz_bw2')
    TopicService()
    print("Ready")

    #TODO test 
    # rospy.Service('start_monitoring', StartMonitoring, handle_start_monitoring)
        

    #TODO test 
    # topic = TopicInfo('/sin', 'std_msgs/Float64')
    # topic = TopicInfo('/cos', 'std_msgs/Float64')
    # topic = TopicInfo('/rosout', 'rosgraph_msgs/Log')

    # monitor = topic.toggle_monitoring() 
    # time.sleep(1)
    # rate, mean, min_delta, max_delta = topic.get_hz()
    # bytes_per_s, mean_size, min_size, max_size = topic.get_bw()

    # time.sleep(1)
    # notmonitor = topic.stop_monitoring()

    # print('--- hz ---')
    # rate_text = '%1.2f' % rate if rate != None else 'unknown'
    # print(rate)
    # print(rate_text)
    
    # print('--- bw ---')
    # if bytes_per_s is None:
    #     bandwidth_text = 'unknown'
    # elif bytes_per_s < 1000:
    #     bandwidth_text = '%.2fB/s' % bytes_per_s
    # elif bytes_per_s < 1000000:
    #     bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
    # else:
    #     bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)    
    # print(bytes_per_s)
    # print(bandwidth_text)
    
    rospy.spin()

if __name__ == "__main__":
    topic_hz_bw2()
