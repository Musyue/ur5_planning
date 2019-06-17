#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy,numpy
from ur5_planning.msg import uv
from ur5_planning.msg import tileuv
import math
from src.ur5_planning.scripts.tile_uv_sub_node import *
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
        tile_reader = TileUvRead()
        tileuv_sub = rospy.Subscriber("/tile_uv", tileuv, tile_reader.callback)
        xp_pub = rospy.Publisher(self.topicname, uv, queue_size=10)
        return xp_pub,tile_reader
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

        h = ((0.15 - rc1) ** 10)/ (b **10) + ((0.15 - rc2) **10) / (b **10) - 1

        w = 1 - (min(0, ((min(0, h)) ** N - (self.kappa ** 10 - 1) **N)))**N / (self.kappa ** 10 - 1) ** (N ** 2)

        xp = numpy.array(xo) - w * a * (numpy.array(x) - numpy.array(xd))

        detax=numpy.array(x)-xp
        print "detax",detax
        flag=0
        if ((detax[0]<=0 and detax[0]>-3) or (detax[0]>=0 and detax[0]<=3) ) and ((detax[1]<=0 and detax[1]>-3) or (detax[1]>=0 and detax[1]<=3)):
            print "-----start manipulator........."
            xp=numpy.array(xo) - (numpy.array(x) - numpy.array(xd))
            a=1
            w=1
            flag=1
            print "a", a
            print "h", h
            print "w", w
            print "xp", xp
            return flag,xp

        print "a", a
        print "h", h
        print "w", w
        print "xp", xp
        return flag,xp
    def get_draw_circle_uv(self,t):
        u = self.uv_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / 100 )
        v = self.uv_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / 100 )
        return  [u,v]

def main():
    #uvlist=[123.0,112.0]
    uvlist=[]
    uvcentral=[331,229]
    uv0=UVpub("uv_design_pub","/xp_uv/xp",uvcentral,100,5,0.7)
    #x=[350,279]
    #x=[313,250]
    # xo=[515,89]
    # xd=[60,365]
    #xp=uv0.get_fov_desire_uv(x,xo,xd)
    xp_pub, tile_reader=uv0.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)
    t=0
    #a=uv()
    b=uv()
    temp = []
    flagg=0
    while not rospy.is_shutdown():
        try:

            if len(tile_reader.tile_0_buf)!=0 and len(tile_reader.tile_1_buf)!=0 and flagg !=1:
                print "all tiles in fov ----\n"
                x=uvcentral
                xo=list(tile_reader.tile_1_buf[-1][1])
                xd=list(tile_reader.tile_0_buf[-1][1])
                print "x,xo,xd", x, xo, xd
                temp=xd
                flag,xp=uv0.get_fov_desire_uv(x,xo,xd)
                flagg=flag
                b.uvinfo = xp
                xp_pub.publish(b)
                print "publish xp uv ------\n",xp
            elif len(tile_reader.tile_0_buf)==0 and len(tile_reader.tile_1_buf)==0:
                print "please wait the tile uv subscribe  ok --------\n"
                pass
            else:
                print "just one tile in fov ----\n"
                x = uvcentral
                xo=list(tile_reader.tile_0_buf[-1][1])
                xd=list(tile_reader.tile_0_buf[-1][1])
                print "x,xo,xd", x, xo, xd
                flag,xp = uv0.get_fov_desire_uv(x, xo, xd)
                if flag==1:
                    if len(tile_reader.tile_1_buf)!=0:
                        xd=temp
                        print "temp-----",temp
                        print "------------------start publish manipulator------",xd
                        xp = xo
                        flagg=flag
                        time.sleep(4)
                        print "flagg----", flagg
                    else:
                        xd=temp
                        print "temp-----",temp
                        print "------------------start publish manipulator------",xd
                        flag, xp = uv0.get_fov_desire_uv(x, xo, xd)
                        flagg=flag
                        print "flagg----", flagg

                b.uvinfo = xp
                xp_pub.publish(b)
                print "publish xp uv ------\n", xp

        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

