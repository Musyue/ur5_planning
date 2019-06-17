#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy
from ur5_planning.msg import uv
import math
class UVpub():
    def __init__(self,nodename,topicname,uv_center_pos,radius,cont):
        self.nodename=nodename
        self.topicname=topicname
        self.uv_center_pos=uv_center_pos
        self.radius=radius
        self.cont=cont  ######
        rospy.init_node(self.nodename)
    def Init_node(self):
        ur_pub = rospy.Publisher(self.topicname, uv, queue_size=10)
        return ur_pub

    def get_draw_circle_uv(self,t):
        u = self.uv_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont )
        v = self.uv_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)
        return  [u,v]#[327, 257]#
    def get_draw_line_uv(self, flag):
        l1 = range(0,199,2)
        l2 = range(199,0,-2)
        l = l1 + l2
        u = self.uv_center_pos[0]
        v = self.uv_center_pos[1]-100+ l[flag]
        return  [u,v]
def main():
    #uvlist=[123.0,112.0]316,251
    uvlist=[]
    uvcentrallist=[336,259]#[327, 257]
    radius = 150
    ratet = 9  # 1
    cont = 400#  500  perfect for no IMU
    uv0=UVpub("uv_design_pub","/camera_uv/uvlist",uvcentrallist,radius,cont)
    ur_pub=uv0.Init_node()
    rate = rospy.Rate(ratet)
    t=0
    a=uv()
    for i in range(uv0.cont):
        uvlist.append(uv0.get_draw_circle_uv(i))
        # uvlist.append(uv0.get_draw_line_uv(i % 200))
    print uvlist
    while not rospy.is_shutdown():
        # ur_pub.publish(a)
        # print uvlist[0]
        try:
            a.uvinfo = uvlist[t%uv0.cont]
            ur_pub.publish(a)
            print  "---------------\n",t%uv0.cont,uvlist[t%uv0.cont]
            t += 1
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

