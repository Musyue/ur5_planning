#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt16
import time
class LedstateRead():
    def __init__(self):
       # self.nodename=nodename
        self.ledstate_buf=[]
        self.checknum=0


    def Init_node(self):
        rospy.init_node("ledstate_node")
        sub = rospy.Subscriber("/led_state", UInt16, self.callback)
        return sub
    def callback(self,msg):
        if len(self.ledstate_buf)==100:
            self.ledstate_buf=self.ledstate_buf[1:]
            self.ledstate_buf.append(msg.data)
        else:
            self.ledstate_buf.append(msg.data)

def main():
    uv0=LedstateRead()
    uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.ledstate_buf)==0:
            print "wait data----\n"
            continue
        else:
            time.sleep(1)
            print "------ledstate_buf--------\n",uv0.ledstate_buf[-1],len(uv0.ledstate_buf)



        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()