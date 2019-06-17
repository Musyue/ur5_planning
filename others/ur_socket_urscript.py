#! /usr/bin/env python
import socket
import sys
import signal

import rospy
import std_msgs.msg


def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


class URScriptDriver(object):
    """
    This class is intended to simply forward a received string to a UR5
    """
    def __init__(self):
        """
        Default constructor
        """
        self.HOST = rospy.get_param("~robot_ip", "192.168.1.2")  # The remote host
        self.PORT = rospy.get_param("~urscript_port", 30002)  # The same port used by the server
        self.urscript_topic = rospy.get_param("~urscript_topic", "/ur_driver/URScript")  # ROS Interface [in]

        # Starting communication
        rospy.logdebug("[URScriptDriver.__init__] Starting communication: %s:%d" % (self.HOST, self.PORT))
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.HOST, self.PORT))
        rospy.sleep(0.05)

        # Starting ROS interface
        self.sub = rospy.Subscriber(self.urscript_topic, std_msgs.msg.String, self.cb_urscript_received )

    def __del__(self):
        """
        Default Destructor
        :return: -
        :rtype: -
        """
        self.s.close()

    def cb_urscript_received(self, msg):
        """
        Receiving a URScript program that should be forwarded to the UR-Robot
        :param msg: URScript program
        :type msg: std_msgs.String
        :return: -
        :rtype: -
        """
        rospy.loginfo("[URScriptDriver.cb_urscript_received] Sending: %s", msg.data)
        self.s.send(msg.data+"\n")
        data = self.s.recv(1024)
        rospy.logdebug("[URScriptDriver.cb_urscript_received] Received: %s", repr(data))


if __name__ == '__main__':
    rospy.init_node("URScript_driver", log_level=rospy.INFO)
    driver = URScriptDriver()

    while not rospy.is_shutdown():
        rospy.spin()
    rospy.loginfo("[URScript_driver.main] Shutting down")