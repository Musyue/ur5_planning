#!/usr/bin/env python
import rospy
from ur5_planning.msg import tileuv
from std_msgs.msg import UInt16
from src.ur5_planning.others.led_state_sub import *
import time
class TileUvRead():
    def __init__(self):
       # self.nodename=nodename
        self.tile_0_buf=[]
        self.checknum=0
        self.tile_1_buf = []
        self.ledstate=None
        self.changeuv=0

    def Init_node(self):
        rospy.init_node("tileuv_node")
        sub = rospy.Subscriber("/tile_uv", tileuv, self.callback)
        # led13 = LedstateRead()
        # led_sub=rospy.Subscriber("/led_state", UInt16, led13.callback)
        # if len(led13.ledstate_buf)!=0:
        #     self.ledstate=led13.ledstate_buf[-1]
        led13 = LedstateRead()
        led_sub=rospy.Subscriber("/led_state", UInt16, led13.callback)

        return sub,led13
    def callback(self,msg):
        led13 = LedstateRead()
        led_sub=rospy.Subscriber("/led_state", UInt16, led13.callback)
        print "self.ledstate", led13.ledstate_buf
        if len(led13.ledstate_buf)!=0:
            self.ledstate=led13.ledstate_buf[-1]
            #print "self.ledstate",led13.ledstate_buf[-1]
        if msg.tile_id ==0:
            if len(self.tile_0_buf)==10:
                self.tile_0_buf=self.tile_0_buf[1:]
                tile_id=msg.tile_id
                cen_uv=msg.cen_uv
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.tile_0_buf.append([tile_id,cen_uv.uvinfo,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
                #print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id=msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.tile_0_buf.append([tile_id,cen_uv.uvinfo,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
            self.checknum = 0

        elif msg.tile_id == 1:
            if len(self.tile_1_buf)==10:
                self.tile_0_buf=self.tile_0_buf[1:]
                tile_id=msg.tile_id
                cen_uv=msg.cen_uv
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.tile_1_buf.append([tile_id,cen_uv.uvinfo,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
                #print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id=msg.tile_id
                cen_uv = msg.cen_uv
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.tile_1_buf.append([tile_id,cen_uv.uvinfo,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
            self.checknum = 1
        elif msg.tile_id==None and self.ledstate==1:
            time.sleep(2)
            self.tile_0_buf = []
            self.changeuv=1
        else:
            print "wait opencv caulate tile uv ----"
            time.sleep(1)
        if self.checknum!=1:
            self.tile_1_buf=[]
        print "checknum",self.checknum
        print "ledstate----------",self.ledstate
def main():
    uv0=TileUvRead()
    sub,led13=uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.tile_0_buf)==0 and len(uv0.tile_1_buf)==0:
            print "wait data----\n"
            pass
        else:
            time.sleep(1)
            print "------tile_0_buf--------\n",uv0.tile_0_buf
            print "------tile_1_buf--------\n", uv0.tile_1_buf
        print "ledstate",led13.ledstate_buf


        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()