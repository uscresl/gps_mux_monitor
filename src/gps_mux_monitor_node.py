#!/usr/bin/python

"""
This file defines a service node that subscribes to all topics that a mux does,
and monitors GPS sensors to select one from them based on their health.
"""

PKG = "gps_mux_monitor"
TOPICS_TIMEOUT = 100
INF = float('inf')

import rospy
import roslib; roslib.load_manifest(PKG)
from sensor_msgs import NavSatFix
from topic_tools.srv import MuxList, MuxSelect
from std_msgs.msg import String as rospy_str
import time


class GPSSubscriber:
    """
    Subscribes to a GPS sensor and maintains the latest message received from there.
    """
    def __init__(self, topic):
        self.topic = topic
        self.subscriber = rospy.Subscriber(topic, NavSatFix, self.callback)
        self.latest_message = None
        self.latest_message_time = None

    def callback(self, navsat_fix_msg):
        self.latest_message = navsat_fix_msg
        self.latest_message = time.time()

    def covariance(self):
        if self.latest_message is None:
            return INF
        return sum(self.latest_message.position_covariance)

    def has_fix(self):
        if self.latest_message is None:
            return False
        return self.latest_message.status.status >= self.latest_message.status.STATUS_FIX

    def __del__(self):
        self.subscriber.unregister()


class GPSMonitorNode:
    """
    This class monitors GPS Sensors and instructs a mux to publish a certain topic based on their health.
    """
    def __init__(self, mux_service_name, switch_delay, timeout, covariance_threshold):
        self.mux_service_name = mux_service_name
        if self.mux_service_name[-1] != '/':
            self.mux_service_name += '/'
        self.switch_delay = switch_delay
        self.timeout = timeout
        self.covariance_threshold = covariance_threshold
        self.subscribed_topics = list()
        self.selected_topic = None
        self.selected_topic_on_top = INF
        self.selected_topic_subscriber = rospy.Subscriber(self.mux_service_name + 'selected',
                                                          rospy_str, self.selected_topic_callback)
        self.subscribers_map = dict()  # This will maintain subscribers to all the topics

        self.spin()

    def spin(self):
        r = rospy.Rate(1)
        topics_timeout = 0

        while not rospy.is_shutdown():
            r.sleep()
            if not (self.selected_topic is None or len(self.subscribed_topics) == 0):
                # Then we have information about the topic the mux listens to, and the list of topics is non-empty
                self.set_best_sensor()

            if self.selected_topic is None or len(self.subscribed_topics) == 0 or topics_timeout >= TOPICS_TIMEOUT:
                # Then we need to update the topics list and change the subscribed sensor if required
                topics_timeout = 0
                topics = self.mux_topic_list()
                if topics is not None:
                    # Subscribe to these topics
                    topic_map = dict()
                    for topic in topics:
                        topic_map[topic] = True
                        if topic not in self.subscribers_map:
                            subscriber = GPSSubscriber(topic)
                            self.subscribers_map[topic] = subscriber
                    for topic in self.subscribers_map:
                        if topic not in topic_map:
                            subscriber = self.subscribers_map.pop(topic)
                            del subscriber

                    self.subscribed_topics = topics
                self.set_best_sensor()

            topics_timeout += 1

    def set_best_sensor(self):
        # Based on covariance, timeout, switch delay, fix etc set self.selected_topic
        covariances = [(sensor.topic, sensor.covariance()) for sensor in self.subscribers_map.values()]
        covariances.sort(key=lambda c: c[1])  # Sort according to covariance
        select_topic = self.selected_topic

        if self.selected_topic is not None:
            # We monitor the current best and change if necessary
            sensor = self.subscribers_map[self.selected_topic]
            if not sensor.has_fix():
                self.selected_topic = None
            elif (time.time() - sensor.latest_message_time) >= self.timeout:
                # Then the last message we received from this sensor is pretty old
                self.selected_topic = None
            if self.selected_topic == covariances[0][0] and sensor.covariance() <= self.covariance_threshold:
                self.selected_topic_on_top = 0
            else:
                self.selected_topic_on_top += 1
            if self.selected_topic_on_top >= self.switch_delay:
                # Then our 'best' sensor has not been the best for a while. We need to switch
                self.selected_topic = None

        if self.selected_topic is None:
            # Then we have to select one from the available
            for (topic, covariance) in covariances:
                if self.subscribers_map[topic].has_fix():
                    select_topic = topic
                    break

        if select_topic != self.selected_topic:
            # Then make the mux select the new topic
            self.mux_select(select_topic)

    def mux_select(self, topic):
        select_service_name = self.mux_service_name + 'select'
        rospy.wait_for_service(select_service_name)
        try:
            service_call = rospy.ServiceProxy(select_service_name, MuxSelect)
            select_arg = MuxSelect(topic=topic)
            service_call(select_arg)

            self.selected_topic = topic
            self.selected_topic_on_top = 0
        except rospy.ServiceException as e:
            self.selected_topic = None
            self.selected_topic_on_top = INF
            rospy.logerr("Failed to make mux select topic: %s - %s" % (topic, str(e)))

    def mux_topic_list(self):
        list_service_name = self.mux_service_name + 'list'
        rospy.wait_for_service(list_service_name)
        try:
            service_call = rospy.ServiceProxy(list_service_name, MuxList)
            response = service_call()
            if response is not None:
                return response.topics
        except rospy.ServiceException as e:
            rospy.logerr("Failed to list mux topics: %s" % str(e))
        return None

    def selected_topic_callback(self, topic):
        self.selected_topic = str(topic)


if __name__ == "__main__":
    try:
        mux_service_name = rospy.get_param('~mux_service_name')
        switch_delay = int(rospy.get_param('~switch_delay'))
        timeout = int(rospy.get_param('~timeout'))
        covariance_threshold = float(rospy.get_param('~covariance_threshold'))
        g = GPSMonitorNode(mux_service_name, switch_delay, timeout, covariance_threshold)
    except KeyError as e:
        print "Missing argument. Usage: _mux_service_name:=<str> _switch_delay:=<int_seconds> _timeout:=<int_seconds>"\
            + "_covariance_threshold:=<float_threshold>"
        print e
    except ValueError as e:
        print "switch_delay and timeout must be integers"
        print e
