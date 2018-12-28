#!/usr/bin/env python
import rospy
from ur5_planning.msg import uv
import time
class UVRead():
    def __init__(self):
       # self.nodename=nodename
        self.uvlist_buf=[]
    def Init_node(self):
        rospy.init_node("desireuv_node")
        sub = rospy.Subscriber("/camera_uv/uvlist", uv, self.callback)
        return sub
    def callback(self,msg):
        if len(self.uvlist_buf)==100:
            self.uvlist_buf=self.uvlist_buf[1:]
            self.uvlist_buf.append(list(msg.uvinfo))
            self.uvlist_buf.append(list(msg.uvinfo))
            #print "---------self.cross_uv_buf",self.cross_uv_buf

        else:
            self.uvlist_buf.append(list(msg.uvinfo))
def main():
    uv0=UVRead()
    uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.uvlist_buf)==0:
            print "wait data----\n"
            continue
        else:
            time.sleep(1)
            print "--------------\n",uv0.uvlist_buf[-1]
            print "length---------------\n",len(uv0.uvlist_buf)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()