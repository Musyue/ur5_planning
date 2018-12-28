#!/usr/bin/env python
import rospy
from ur5_planning.msg import uv
import time
class CROSSLINEUVRead():
    def __init__(self):
       # self.nodename=nodename
        self.cross_uv_buf=[]
    def Init_node(self):
        rospy.init_node("desireuv_node")
        crossline_sub = rospy.Subscriber('/intersection_uv', uv, self.callback_cross_line)
        return crossline_sub
    def callback_cross_line(self,msg):
        if len(self.cross_uv_buf)==100:
            self.cross_uv_buf= self.cross_uv_buf[1:]
            self.cross_uv_buf.append(list(msg.uvinfo))
            self.cross_uv_buf.append(list(msg.uvinfo))
            #print "---------self.cross_uv_buf",self.cross_uv_buf

        else:
            self.cross_uv_buf.append(list(msg.uvinfo))
def main():
    uv0=CROSSLINEUVRead()
    uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.cross_uv_buf)==0:
            print "wait data----\n"
            continue
        else:
            time.sleep(1)
            print "--------------\n",uv0.cross_uv_buf[-1]
            print "length---------------\n",len(uv0.cross_uv_buf)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()