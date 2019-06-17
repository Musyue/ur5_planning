#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String
import time
rospy.init_node("move_ur5_by_urscript")
pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
rate=rospy.Rate(30)
def change_angle_to_pi(qangle):
    temp=[]
    for i in xrange(len(qangle)):
        temp.append(qangle[i]/180.0*3.14)
    return temp
def moveur(pub,q,ace,vel,t):
    ss="movej(["+str(q[0])+","+str(q[1])+","+str(q[2])+","+str(q[3])+","+str(q[4])+","+str(q[5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
    print ss
    pub.publish(ss)
while not rospy.is_shutdown():
    t=0
    vel=0.2
    ace=50
    qq=[
        [99.08,-111.84,-79.77,188.27,-81.97,226.91]
        # [90.65,-118.06,-56.76,170.16,-94.45,229.83]
        #[-94,-95.29,-90.44,-86.80,91.62,71.55]
        # [307.61, -136.58, 65.96, -120.66, -306.25, -215.74]
        # [182.22,-141.28,108.32,-137.17,-180.82,-218.81]#ok
        # [266.09, -178.29, 117.69, -121.76, -267.38, -216.20]
        #202.22,-199.82

        # [86.02,-140.65,112.10,-148.28,-88,-320.66]
        # [35.78,-105.15,87.76,-235.20,-90.99,195.06]
        # [93.93,-145.95,100.84,-121.30,-91.04,-326.16]
        # [34.26,-99.78,81.03,-239.41,-94.51,-184.10]
        # [75.59,-0.51,-88.94,-188.57,-75.17,148.67]
        # [267,-46,-127.23,-94.8,-79,-132.34]
        # [93,122,-75,-81,-110,-37],
        # [90, -62, -104, -105, -110, -34]
        # [42.27,-105.20,87.8,-212.62,-86.5,195.16]
        # [65.38,-108.78,94.23,-254.23,-75.22,166.93]
        ]
    for ii in xrange(len(qq)):
        qt=change_angle_to_pi(qq[ii])
        # time.sleep(1)
        # qt=[3.1897695779800417, -2.471768395100729, 1.8933079242706299, -2.4048668066607877, -3.1475941101657314, -3.8097620646106165]
        moveur(pub, qt,ace,vel,t)
        # time.sleep(1)
    rate.sleep()
