#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time,sys
import rospy,numpy
from ur5_planning.msg import uv
from ur5_planning.msg import tileuv
from std_msgs.msg import UInt16
import math
from std_msgs.msg import String
from src.ur5_planning.scripts.tile_uv_sub_node import *
from std_msgs.msg import UInt16
from led_state_sub import *
from src.ur5_planning.scripts.frompitoangle import *
class UVpub():
    def __init__(self,nodename,topicname,uv_center_pos,radius,delta,kappa):
        self.nodename=nodename
        self.topicname=topicname
        self.uv_center_pos=uv_center_pos
        self.radius=radius
        self.delta=delta
        self.kappa=kappa

        self.tile_0_buf=[]
        self.checknum=0
        self.tile_1_buf = []
        self.ledstate=None
        self.changeuv=None

    def Init_node(self):
        rospy.init_node(self.nodename)
        # tile_reader = TileUvRead()
        tileuv_sub = rospy.Subscriber("/tile_uv", tileuv, self.callback)
        xp_pub = rospy.Publisher(self.topicname, uv, queue_size=10)
        led13 = LedstateRead()
        led_sub=rospy.Subscriber("/led_state", UInt16, led13.callback)

        return xp_pub,led13
    """
    0:o
    1:d
    """
    def callback(self, msg):
        if msg.tile_id == 0:
            if len(self.tile_0_buf) == 10:
                self.tile_0_buf = self.tile_0_buf[1:]
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_0_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
                # print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_0_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
            self.checknum = 0

        elif msg.tile_id == 1:
            if len(self.tile_1_buf) == 10:
                self.tile_1_buf = self.tile_1_buf[1:]
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_1_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
                # print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id = msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv = msg.f1th_uv
                s2th_uv = msg.s2th_uv
                t3th_uv = msg.t3th_uv
                f4th_uv = msg.f4th_uv
                self.tile_1_buf.append(
                    [tile_id, cen_uv.uvinfo, f1th_uv.uvinfo, s2th_uv.uvinfo, t3th_uv.uvinfo, f4th_uv.uvinfo])
            self.checknum = 1
        # elif msg.tile_id == None and self.ledstate == 1:
        #     time.sleep(2)
        #     self.tile_0_buf = []
        #     self.changeuv = 1
        #     print "hhhhhhhhhhhhhhhhhhhhhhh-------"
        else:
            print "wait opencv caulate tile uv ----"
            time.sleep(1)
        #print " msg.tile_id", msg.tile_id
        if self.checknum != 1:
            self.tile_1_buf = []
        #print "checknum", self.checknum
        # print "ledstate----------", self.ledstate
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
    led13:high pick tile,and high data=1,low place tile
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
        # if ((detax[0]<=0 and detax[0]>-3) or (detax[0]>=0 and detax[0]<=3) ) and ((detax[1]<=0 and detax[1]>-3) or (detax[1]>=0 and detax[1]<=3)):
        #     print "-----start manipulator........."
        #     xp=numpy.array(xo) - (numpy.array(x) - numpy.array(xd))
        #     a=1
        #     w=1
        #     flag=1
        #     print "a", a
        #     print "h", h
        #     print "w", w
        #     print "xp", xp
        #     return flag,xp
        print "a", a
        print "h", h
        print "w", w
        print "xp", xp.tolist()
        return xp.tolist()
    def get_fov_desire_uv_0(self,xp):
        a=1
        print "a",a
        return xp
    #qq=[12,11,....]angular joint

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
    xp_pub,led13=uv0.Init_node()
    ratet=1
    rate = rospy.Rate(ratet)

    #a=uv()
    b=uv()
    temp = []
    xpp=[]
    flagg=0
    while not rospy.is_shutdown():
        if len(led13.ledstate_buf):
            uv0.ledstate=led13.ledstate_buf[-1]
        try:
            if flagg==1:
                if len(uv0.tile_0_buf)==0 and len(uv0.tile_1_buf)==0:
                    print "please wait the new tile uv subscribe  ok --------\n"
                    time.sleep(3)
                    continue
                if len(uv0.tile_0_buf)!=0 and len(uv0.tile_1_buf)!=0 and uv0.ledstate==1:
                    print "second ---- all tiles in fov ----\n"
                    x=uvcentral
                    #right corner
                    xo=list(uv0.tile_0_buf[-1][1])
                    xd=list(uv0.tile_1_buf[-1][1])
                    xp=xd
                    b.uvinfo = xp
                    xp_pub.publish(b)
                    print "second xp,xd xd ----------",xp
                else:
                    try:
                        print "second just one tile in fov ----\n"
                        print " uv0.tile_0_buf=[], uv0.tile_1_buf=[]", uv0.tile_0_buf, "\n",uv0.tile_1_buf
                        x = uvcentral
                        xo=list(uv0.tile_0_buf[-1][1])
                        xd=list(uv0.tile_0_buf[-1][1])
                        print "x,xo,xd", x, xo, xd
                        xp = uv0.get_fov_desire_uv(x, xo, xd)
                        b.uvinfo = xo
                        xp_pub.publish(b)
                        print "publish xp uv ------\n", xp
                    except:
                        pass
            # print "tile_reader",uv0.changeuv
            else:
                if len(uv0.tile_0_buf)==0 and len(uv0.tile_1_buf)==0:
                    print "please wait the tile uv subscribe  ok --------\n"
                    pass
                elif len(uv0.tile_0_buf)!=0 and len(uv0.tile_1_buf)!=0 and uv0.ledstate==0:
                    print "first ---- all tiles in fov ----\n"
                    x=uvcentral
                    #left corner
                    xo=list(uv0.tile_0_buf[-1][1])
                    xd=list(uv0.tile_1_buf[-1][1])
                    print "x,xo,xd", x, xo, xd
                    if len(temp)==10:
                        temp=temp[:(len(temp)-1)]
                        temp.append(xd)
                    else:
                        temp.append(xd)
                    if len(temp)==1:
                        xpp=temp[0]
                    xp=uv0.get_fov_desire_uv(x,xo,xd)
                    b.uvinfo = xp
                    xp_pub.publish(b)
                    print "temp------\n",temp
                    print "xpp------\n", xpp
                    print "publish xp uv ------\n",xp
                elif len(uv0.tile_0_buf)==10 and len(uv0.tile_1_buf)==0 and uv0.ledstate==1:
                    # time.sleep(3)
                    print "pick and manipulator-------"
                    try:
                        x = [300,249]
                        xo = list(uv0.tile_0_buf[-1][1])
                        xd = xpp
                        #print "x,xo,xd", x, xo, xd
                        xp=numpy.array(xo) - (numpy.array(x) - numpy.array(xd))
                        b.uvinfo = xp.tolist()
                        xp_pub.publish(b)
                        print "publish xp uv ------", xp
                        u_error=abs(331-xp.tolist()[0])
                        v_error=abs(219-xp.tolist()[1])

                        if (u_error<=6 and u_error>=0) and (v_error<=33 and v_error>=0):
                            flagg=1
                            uv0.tile_0_buf=[]
                            uv0.tile_1_buf=[]
                        print "flagg-------",flagg
                        print "u_error,v_error", u_error,v_error
                        print " uv0.tile_0_buf=[]", uv0.tile_0_buf
                        print ",uv0.tile_1_buf=[]",uv0.tile_1_buf
                    except:
                        pass
                else:
                    print "just one tile in fov ----\n"
                    x = uvcentral
                    xo=list(uv0.tile_0_buf[-1][1])
                    xd=list(uv0.tile_0_buf[-1][1])
                    print "x,xo,xd", x, xo, xd
                    xp = uv0.get_fov_desire_uv(x, xo, xd)
                    b.uvinfo = xp
                    xp_pub.publish(b)
                    print "publish xp uv ------\n", xp
                # print "tile_reader",uv0.changeuv
        except KeyboardInterrupt:
            sys.exit()

        rate.sleep()
if __name__=="__main__":
    main()

