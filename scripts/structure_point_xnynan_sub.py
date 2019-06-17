#!/usr/bin/env python
import rospy
# from ur5_planning.msg import structure_point
from std_msgs.msg import UInt16
from std_msgs.msg import Float64
import time
class StructurePointxnynanRead():
    def __init__(self):
       # self.nodename=nodename
        self.sturucture_point_xn_buf=[]

        self.sturucture_point_yn_buf = []

        self.sturucture_point_an_buf = []
        self.checknum=0


    def Init_node(self):
        rospy.init_node("structure_point_node")
        xn_sub = rospy.Subscriber("/cross_line_xsubn", Float64, self.structure_point_xn_callback)
        yn_sub = rospy.Subscriber("/cross_line_ysubn", Float64, self.structure_point_yn_callback)

        an_sub = rospy.Subscriber("/cross_line_asubn", Float64, self.structure_point_an_callback)
        return xn_sub,yn_sub,an_sub
    def structure_point_xn_callback(self,msg):

        if len(self.sturucture_point_xn_buf)==10:
            self.sturucture_point_xn_buf=self.sturucture_point_xn_buf[1:]
            xn=msg.data
            self.sturucture_point_xn_buf.append(xn)
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            xn=msg.data
            self.sturucture_point_xn_buf.append(xn)
    def structure_point_yn_callback(self,msg):

        if len(self.sturucture_point_yn_buf)==10:
            self.sturucture_point_yn_buf=self.sturucture_point_yn_buf[1:]
            yn=msg.data
            self.sturucture_point_yn_buf.append(yn)
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            yn=msg.data
            self.sturucture_point_yn_buf.append(yn)
    def structure_point_an_callback(self,msg):

        if len(self.sturucture_point_an_buf)==10:
            self.sturucture_point_an_buf=self.sturucture_point_an_buf[1:]
            an=msg.data
            self.sturucture_point_an_buf.append(an)
            #print "---------self.cross_uv_buf",self.cross_uv_buf
        else:
            an=msg.data
            self.sturucture_point_an_buf.append(an)

def main():
    uv0=StructurePointxnynanRead()
    sub=uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.sturucture_point_xn_buf)==0:
            print "wait data----\n"
            pass
        else:
            time.sleep(1)

        print "sturucture_point_xn_buf------",uv0.sturucture_point_xn_buf
        print "sturucture_point_yn_buf------", uv0.sturucture_point_yn_buf
        print "sturucture_point_an_buf------", uv0.sturucture_point_an_buf
        time.sleep(1)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()