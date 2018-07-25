#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Empty
#from sensor_msgs.msg import JointState
from frompitoangle import *
import os,time
class PickPlace():
    rospy.init_node("move_ur5_by_urscript")
    def __init__(self):
        pass
    def Init_node(self):
        pub=rospy.Publisher("/ur_driver/URScript",String,queue_size=10)
        return pub
    #qq is list,you must instance it by list
    def init_nano(self):
        rospy.loginfo('Waiting for nano drive relay...')
        #drive arduino nano
        #rospy.init_node("drive_arduino_nano")
        pub=rospy.Publisher("toggle_led",Empty,queue_size=2)
        #rate=rospy.Rate(2)
        #return pub,rate
    def pub_ledstate(self):
        os.system("rostopic pub /toggle_led std_msgs/Empty '{}' --once")
    def init_and_pub(self,qq,vel,ace,t):
        self.init_nano()
        rate=rospy.Rate(0.07)
        pub=self.Init_node()
        while not rospy.is_shutdown():
            #t=3
            #vel=1.0
            #ace=1.0
            #q=[-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223]
            # rospy.loginfo('pick the title.....')
            # self.pub_ledstate()
            # print "this is the first planning--------\n"
            # rospy.loginfo('pick the title done.....')
            rospy.loginfo('Moving the arm to goal position...\n it will pick from first position to the 9th position')
            for i in range(len(qq)):    
                ss="movej(["+str(qq[i][0])+","+str(qq[i][1])+","+str(qq[i][2])+","+str(qq[i][3])+","+str(qq[i][4])+","+str(qq[i][5])+"]," +"a="+str(ace)+","+"v="+str(vel)+","+"t="+str(t)+")"
                print "--------",i+1,"th postion!watch out!-------------\n",ss
                #ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
                pub.publish(ss)
                
                if i==1:
                    rospy.loginfo('pick the title.....')
                    self.pub_ledstate()
                    print "this is the first planning--------\n"
                    rospy.loginfo('pick the title done.....')
                    #rospy.loginfo('Moving the arm to goal position...,it will pick from first position to the 9th position')
                if i==10:
                    rospy.loginfo('...done')
                    rospy.loginfo('place the title.....')
                    os.system("rostopic pub /toggle_led std_msgs/Empty '{}' --once")
                    rospy.loginfo('place the title done ....')
                rate.sleep()
                #rostopic pub /ur_driver/URScript std_msgs/String "movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.4, v=1.0)"
        #time.sleep(3)
    def get_pos(self):
        pass

def main():
    t=1.8
    vel=1.0
    ace=1.0
    #angle
    q1=[194.82,-78.91,-111.49,8.09,-283.35,324.67]
    qpi=[194.82,-80.70,-108.89,8.09,-282.35,324.67]#pick
    q2=[194.82,-78.93,-111.57,8.09,-283.35,134.53]
    q3=[216.96,-78.66,-108.15,7.02,-304.74,134.54]
    q4=[218.20,-81.69,-81.74,-21.13,-305.89,116.59]
    q5=[220.20,-84.46,-77.54,-20.72,-305.26,195.16]
    q6=[214.19,-84.37,-73.42,-20.73,-305.04,266.76]
    q7=[192.69,-84.62,-58.05,-38.86,-282.29,315.46]
    q8=[188.36,-79.86,-85.68,-14.83,-275.64,352.44]
    q9=[188.36,-79.86,-85.68,-14.83,-275.64,110.10]
    q10=[188.36,-83.10,-85.81,-14.83,-275.64,115.24]#place
    q11=[188.36,-79.86,-85.68,-14.83,-275.64,110.10]
    #pi
    pii=[q1,qpi,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11]
    qq=[]
    for i in pii:
        qq.append(getpi_for_py(getpi(i)))
        #getpi_for_py(getpi(q11)),getpi_for_py(getpi(q2)),getpi_for_py(getpi(q3)),getpi_for_py(getpi(q4)),getpi_for_py(getpi(q5)),getpi_for_py(getpi(q6)),getpi_for_py(getpi(q7)),getpi_for_py(getpi(q8)),getpi_for_py(getpi(q9))]
    c0=PickPlace()
    c0.init_and_pub(qq,vel,ace,t)
if __name__=="__main__":
    main()