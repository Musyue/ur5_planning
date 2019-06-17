#!/usr/bin/env python
import rospy
from ur5_planning.msg import structure_point
from std_msgs.msg import UInt16

import time
class StructurePointUvRead():
    def __init__(self):
       # self.nodename=nodename
        self.sturucture_point_0_buf=[]
        self.checknum=0


    def Init_node(self):
        rospy.init_node("structure_point_node")
        sub = rospy.Subscriber("/structure_point_uv", structure_point, self.structure_point_callback)


        return sub
    def structure_point_callback(self,msg):
        if msg.tile_id ==0:
            if len(self.sturucture_point_0_buf)==10:
                self.sturucture_point_0_buf=self.sturucture_point_0_buf[1:]
                tile_id=msg.tile_id
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.sturucture_point_0_buf.append([tile_id,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
                #print "---------self.cross_uv_buf",self.cross_uv_buf
            else:
                tile_id=msg.tile_id
                f1th_uv=msg.f1th_uv
                s2th_uv=msg.s2th_uv
                t3th_uv=msg.t3th_uv
                f4th_uv=msg.f4th_uv
                self.sturucture_point_0_buf.append([tile_id,f1th_uv.uvinfo,s2th_uv.uvinfo,t3th_uv.uvinfo,f4th_uv.uvinfo])
        else:
            print "wait opencv caulate tile uv ----"
            time.sleep(1)

def main():
    uv0=StructurePointUvRead()
    sub=uv0.Init_node()
    while not rospy.is_shutdown():
        if len(uv0.sturucture_point_0_buf)==0:
            print "wait data----\n"
            pass
        else:
            time.sleep(1)

        print "sturucture_point_0_buf------",uv0.sturucture_point_0_buf
        time.sleep(1)
        #uv0.uvlist=[]
    #rospy.spin()
if __name__=="__main__":
    main()