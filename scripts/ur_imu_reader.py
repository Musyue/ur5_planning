#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import sys
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur5_planning.msg import IMUoutvel
from trans_methods import *
import Quaternion as Q
import transfer as T

"""
function: imu data reader
note: pls use the code from the github, every time the imu node breaks down, you may redownload the code from
https://github.com/Luminary-S/razor-imu-9dof

here:
imu data, we only read
 1,quaternion position,
 2,angular velocity 
 3,linear acceleration

we also can use but not use here are:
 1, timestamp
 2, orientation covariance
 3, angular velocity covariance
 4, linear acc covariance
"""

class ImuReader():
    def __init__(self, name="imu_info_subscriber"):
        self.name = name
        # self.pos_dict = {}
        self.list_quater = []
        self.list_ang_vel = []
        self.list_linear_acc = []
        self.timestamp = []
        self.l_quater = []
        self.l_ang_vel = []
        self.l_linear_acc = []
        self.l_timestamp = []

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/imu", Imu, self.callback)
        return sub

    def callback(self, msg):
        # print msg
        # first time
        quater_pos_list = self.get_pos_from_msg(msg)
        ang_vel_list = self.get_angular_vel_from_msg(msg)
        linear_acc_list = self.get_linear_acc_from_msg(msg)
        # second time, no filter influence.......need to be revised
        # quater_pos_list_2 = self.get_pos_from_msg(msg)
        # ang_vel_list_2 = self.get_angular_vel_from_msg(msg)
        # linear_acc_list_2 = self.get_linear_acc_from_msg(msg)

        timestamp = self.get_timestamp_from_msg(msg)

        # quater_pos_list = self.one_order_filter( quater_pos_list_1, quater_pos_list_2)
        # ang_vel_list = self.one_order_filter( ang_vel_list_1, ang_vel_list_2 )
        # linear_acc_list = self.one_order_filter( linear_acc_list_1, linear_acc_list_2 )
        self.list_quater.append(quater_pos_list)
        self.list_ang_vel.append(ang_vel_list)
        self.list_linear_acc.append(linear_acc_list)
        self.timestamp.append(timestamp)
        # print("quater:", self.l_quater)
        self.filter_data()

    def average_filter(self, list, NUM = 10 ):
        l_n = []
        print("before list:", list)
        if len(list) > NUM:
            len_element = len(list[0])
            for q in range( 0, len(list)-NUM ):
                for i in range(q, q+NUM):
                    c = list_add(list[q],list[q+1])
                average = [ i/10 for i in c]
                l_n.append(average)
            print("after list:", l_n)
            return l_n
        else:
            print("no change:",list)
            return list

    def average_filter_num(self, list, NUM = 10 ):
        l_n = []
        # print("before list:", list)
        if len(list) > NUM:

            for q in range( 0, len(list)-NUM ):
                index = (q+q+NUM)/2
                average = list[index]
                l_n.append(average)
            print("after list:", l_n)
            return l_n
        else:
            print("no change:",list)
            return list


    def filter_data(self):
        if len(self.l_quater) > 2 :
            # print( "leng:", len(self.list_ang_vel) )
            # self.list_quater = self.average_filter(self.l_quater)
            # self.list_ang_vel = self.average_filter(self.l_ang_vel)
            # self.list_linear_acc = self.average_filter(self.l_linear_acc)
            # self.timestamp = self.average_filter_num(self.l_timestamp)
            quater_pos_list_1 = self.list_quater[-2]
            quater_pos_list_2 = self.list_quater[-1]
            # print "quater"
            quater_pos_list = self.one_order_filter(quater_pos_list_1, quater_pos_list_2)

            ang_vel_list_1 = self.list_ang_vel[-2]
            ang_vel_list_2 = self.list_ang_vel[-1]
            # print "ang_vel"
            ang_vel_list = self.one_order_filter(ang_vel_list_1, ang_vel_list_2)

            linear_acc_list_1 = self.list_linear_acc[-2]
            linear_acc_list_2 = self.list_linear_acc[-1]
            # print "linear_acc"
            linear_acc_list = self.one_order_filter(linear_acc_list_1, linear_acc_list_2)

            self.list_quater[-1] = quater_pos_list
            self.list_ang_vel[-1] = ang_vel_list
            self.list_linear_acc[-1] = linear_acc_list
            # self.list_quater.append(quater_pos_list)
            # self.list_ang_vel.append(ang_vel_list)
            # self.list_linear_acc.append(linear_acc_list)

    def get_timestamp_from_msg(self,msg):
        timestamp = msg.header.stamp
        # self.timestamp.append(timestamp)
        return  timestamp

    def get_pos_from_msg(self,msg):
        quaternion = msg.orientation
        # print "type:", type(quaternion)
        quater_list = [ quaternion.x, quaternion.y, quaternion.z, quaternion.w ]
            # [ quaternion[item] for item in quaternion ]
        # print quaternion
        return quater_list

    def get_angular_vel_from_msg(self,msg):
        ang_vel = msg.angular_velocity
        ang_vel_list = [ ang_vel.x, ang_vel.y, ang_vel.z ]
        # print "ang_vel_list:",ang_vel_list
        return ang_vel_list

    def get_linear_acc_from_msg(self,msg):
        linear_acc = msg.linear_acceleration
        linear_acc_list = [ linear_acc.x, linear_acc.y, linear_acc.z ]
        return  linear_acc_list

    def one_order_filter(self, data_list, newdata_list ):
        res_list = []
        lambda1 = 0.05
        # print "data_list:", data_list
        # print "newdata_list:", newdata_list
        # print len(data_list)
        for i in range( len(data_list) ):
            # print "i:", i
            res_list.append( lambda1 * data_list[i] + (1 - lambda1) * newdata_list[i] )
        # print "res_list:",res_list
        # try:
        #     for i in range(len( data_list )):
        #         res_list[i] = lambda1 * data_list[i] + ( 1 - lambda1 ) * newdata_list[i]
        # except:
        #     print(" length of two lists are not same ")
        return res_list

    def cal_base_vel(self, q, J):
        q_now = q
        # ur_KDL.set_q(q_now)
        # tr = ur_KDL.get_fk_pose()
        jac_ur_ee = J
        print "jac_ur_ee:", jac_ur_ee
        print "==========="
        vel = []
        ang_vel = self.list_ang_vel[-1]
        l_vel = []
        pos1 = self.list_quater[-2]
        pos2 = self.list_quater[-1]
        tr1 = quater2T(pos1)
        tr2 = quater2T(pos2)
        print "tr1:", tr1
        print "tr2:", tr2
        l1 = T.transl(tr1)
        l2 = T.transl(tr2)
        time1 = self.timestamp[-2]
        time2 = self.timestamp[-1]
        print "time1",time1,"time2",time2
        t = time2.to_sec() - time1.to_sec()
        print "t",t
        for i in range(len(pos1) - 1):
            l_vel.append((pos2[i] - pos1[i]) * 1.0 / t)

        vel = l_vel + ang_vel
        print "l_vel:", l_vel
        print "ang_vel:", ang_vel
        print "base vel:", vel

        ee_disturb_vel = vel * jac_ur_ee
        print "ee_distrub_vel:", ee_disturb_vel.reshape((6,1))
        return ee_disturb_vel.reshape((6,1)) , vel
        # print "--------------"
        # pass

def quater2T( q_list ):
    rot = q_list[:3]
    s = q_list[3]
    q0 = Q.quaternion(s, np.mat(rot))
    return q0.tr()

def list_add(a,b):
    c = []
    for i in range(len(a)):
        c.append(a[i]+b[i])
    return c
# def get_delta_joint_angular(j_speed, detat=0.05):
#     # j_speed = self.get_joint_speed( uv_list, z, J, X, feature_error )
#     #print j_speed
#     delta_angular = float(detat)*np.array(j_speed)
#     #print '-------joint_angular-----\n',joint_angular
#     # if det_J < DELTA_J_LIMIT:
#     #     ave = np.mean( delta_angular )
#     #     big_list = list( map( lambda x:x>ave, delta_angular.tolist() ) )
#     #     if sum(big_list) == 1:
#     #         id = big_list.index(1)
#     #         delta_angular[id][0] = 0
#     return delta_angular
#
# def get_joint_angular( qnow, delta_angular ):
#     #result=[]
#     listangular=[]
#     for i in range( len( delta_angular.tolist() ) ):
#         listangular.append( delta_angular.tolist()[i][0]+qnow[i])
#     # print "list",detajoint.tolist()
#     return listangular

def main():
    data = []
    ace = 50
    # vel = 0.1
    # urt = 0
    # # ratet = 30
    # imu_Reader = ImuReader()
    # imu_Reader.Init_node()
    # ur_Reader = URposNode()
    # ur_sub = rospy.Subscriber("/joint_states", JointState, ur_Reader.callback)
    # ur_pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
    # imu_pub = rospy.Publisher("/now_imuvel", IMUoutvel, queue_size=10)
    # ur_KDL = UR_kdl()
    # rate = rospy.Rate(30)
    # while not rospy.is_shutdown():
    #     # if len( imu_Reader.list_linear_acc ) == 500:
    #     #     plt.plot( [ i[0] for i in imu_Reader.list_quater ] )
    #     #     plt.show()
    #     # try:
    #     if len( imu_Reader.list_quater ) > 1:
    #         q_now = ur_Reader.ave_ur_pose
    #         ur_KDL.set_q( q_now )
    #         tr = ur_KDL.get_fk_pose()
    #         jac_ur_ee = tr2jac( tr )
    #         # print "jac_ur_ee:", jac_ur_ee
    #         ee_speed, imuvel = imu_Reader.cal_base_vel( q_now, jac_ur_ee )
    #         imu_pub.publish(imuvel)
    #         J = ur_KDL.get_jacobian(q_now)
    #         j_speed = np.dot(J.I, ee_speed)
    #         q_pub_now = get_joint_angular(q_now, get_delta_joint_angular(j_speed))
    #         ss = "movej([" + str(q_pub_now[0]) + "," + str(q_pub_now[1]) + "," + str(q_pub_now[2]) + "," + str(
    #             q_pub_now[3]) + "," + str(q_pub_now[4]) + "," + str(q_pub_now[5]) + "]," + "a=" + str(
    #             ace) + "," + "v=" + str(vel) + "," + "t=" + str(urt) + ")"
    #         print ss
    #         ur_pub.publish(ss)
    #         rate.sleep()
        # except:
        #     print "first time..."
        # pass

if __name__ == "__main__":
     main()
