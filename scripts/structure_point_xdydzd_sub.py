#!/usr/bin/env python
import rospy
# from ur5_planning.msg import structure_point
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
import time
class StructurePointXdydzdRead():
    def __init__(self):
       # self.nodename=nodename
        self.sturucture_point_xd_buf=[]

        self.sturucture_point_yd_buf = []
        self.checknum=0


    def Init_node(self):
        rospy.init_node("structure_point_node")
        xd_sub = rospy.Subscriber("/structure_xd", Float64, self.structure_point_xd_callback)
        yd_sub = rospy.Subscriber("/structure_yd", Float64, self.structure_point_yd_callback)


        return xd_sub,yd_sub
    def structure_point_xd_callback(self,msg):

        if len(self.sturucture_point_xd_buf)==10:
            self.sturucture_point_xd_buf=self.sturucture_point_xd_buf[1:]
            xd=msg.data
            self.sturucture_point_xd_buf.append(xd)
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            xd=msg.data
            self.sturucture_point_xd_buf.append(xd)
    def structure_point_yd_callback(self,msg):

        if len(self.sturucture_point_yd_buf)==10:
            self.sturucture_point_yd_buf=self.sturucture_point_yd_buf[1:]
            yd=msg.data
            self.sturucture_point_yd_buf.append(yd)
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            yd=msg.data
            self.sturucture_point_yd_buf.append(yd)


def main():
    uv0=StructurePointXdydzdRead()
    sub=uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.sturucture_point_xd_buf)==0:
            print "wait data----\n"
            pass
        else:
            time.sleep(1)

        print "sturucture_point_xd_buf------",uv0.sturucture_point_xd_buf
        print "sturucture_point_yd_buf------", uv0.sturucture_point_yd_buf
        time.sleep(1)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()