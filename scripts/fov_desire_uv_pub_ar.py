#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy,numpy
from ur5_planning.msg import uv
from ar_track_alvar_msgs.msg import AlvarMarkers
import math
from get_arpose_from_ar import *
class UVpub():
    def __init__(self,nodename,topicname,uv_center_pos,radius,delta,kappa):
        self.nodename=nodename
        self.topicname=topicname
        self.uv_center_pos=uv_center_pos
        self.radius=radius
        self.delta=delta
        self.kappa=kappa


    def Init_node(self):
        rospy.init_node(self.nodename) 
        ar_reader = arReader(1)
        ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, ar_reader.callback)
        ur_pub = rospy.Publisher(self.topicname, uv, queue_size=10)
        return ur_pub,ar_sub,ar_reader
    """
    x:real time uv from cv[u,v]
    x0:object uv,[u0,v0]
    xd:desire uv [ud,vd]
    f=(x-x0)'*(x-x0)-detat^2
    mm
    rc1=0.56
    rc2=-0.145
    b=0.1
    detax
    """
    def get_instrinc_param(self):
        data = numpy.array(
            [854.095755, 0.000000, 331.439357, 0.000000, 853.591646, 229.264580, 0.000000, 0.000000, 1.000000])
        instrinc_param = data.reshape((3, 3))
       # print(instrinc_param)
        return instrinc_param
    def get_fov_desire_uv(self,x,xo,xd):
        rc1=0.56
        rc2=-0.145
        b=0.1
        N=4
        #print numpy.mat(numpy.array(x)-numpy.array(xo)).T
        f=numpy.dot(numpy.mat(numpy.array(x)-numpy.array(xo)),(numpy.mat(numpy.array(x)-numpy.array(xo))).T)-self.delta**2
        print "f",f.tolist()[0][0]
        f=f.tolist()[0][0]
        a=1-(min(0,((min(0,f))**N-((self.kappa*self.delta)**2-self.delta**2)**N)))**N/(((self.kappa*self.delta)**2-self.delta**2)**(N**2))
        print "a",a
        h = ((0.15 - rc1) ** 10)/ (b **10) + ((0.15 - rc2) **10) / (b **10) - 1
        print "h",h
        w = 1 - (min(0, ((min(0, h)) ** N - (self.kappa ** 10 - 1) **N)))**N / (self.kappa ** 10 - 1) ** (N ** 2)
        print "w",w
        xp = numpy.array(xo) - w * a * (numpy.array(x) - numpy.array(xd))
        print "xp",xp
        detax=numpy.array(x)-xp
        print "detax",detax
        if ((detax[0]<0 and detax[0]>-8) or (detax[0]>0 and detax[0]<8) ) and ((detax[1]<0 and detax[1]>-8) or (detax[1]>0 and detax[0]<8)):
            print "start manipulator........."
            xp=numpy.array(xo) - (numpy.array(x) - numpy.array(xd))
            return xp
        return xp
    def get_draw_circle_uv(self,t):
        u = self.uv_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / 100 )
        v = self.uv_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / 100 )
        return  [u,v]
    def get_uv_from_ar(self,pos):
        #pos = [-0.0694628511461, 0.0487799361822, 0.988924230718]
        # print("pos:", pos)
        cam_pos = numpy.array( pos )
        # 归一化
        # print("cam pos1:", cam_pos)rate = rospy.Rate(0.1)
        # cam_pos = cam_pos.reshape((3,1)) / cam_pos[2]
        cam_pos = cam_pos.T / cam_pos[2]
        # print(cam_pos)
        # print("cam pos2:", cam_pos)
        imgpos = numpy.dot( self.get_instrinc_param(), cam_pos)
        #print imgpos
        imgpos = imgpos[0:2]
        #print("imgps2:", imgpos)
        return imgpos.tolist()
def main():
    #uvlist=[123.0,112.0]
    uvlist=[]
    uvcentrallist=[313,250]
    uv0=UVpub("uv_design_pub","/xp_uv/xp",uvcentrallist,100,5,0.7)
    #x=[350,279]
    #x=[313,250]
    xo=[515,89]
    xd=[60,365]
    #xp=uv0.get_fov_desire_uv(x,xo,xd)
    ur_pub,ar_sub,ar_reader=uv0.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)
    t=0
    #a=uv()
    b=uv()
    #for i in range(100):
    #    uvlist.append(uv0.get_draw_circle_uv(i))
    #print uvlist
    while not rospy.is_shutdown():
        # ur_pub.publish(a)
        # print uvlist[0]
        try:
     #       a.uvinfo = uvlist[t%100]
            pos_dict = ar_reader.ave_pos_dict
            if len(pos_dict)!=0:
                x=uv0.get_uv_from_ar(pos_dict[0][:3])[:2]
                print "now uv -------\n",x
                xp=uv0.get_fov_desire_uv(x,xo,xd)
                b.uvinfo = xp 
     #       ur_pub.publish(a)
                ur_pub.publish(b)
            #print  "---------------\n",t%100,uvlist[t%100]
            #t += 1
                print "publish xp uv ------\n",xp
            else:
                print "please wait the ar marker ok --------\n"
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

