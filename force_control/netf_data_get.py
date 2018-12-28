#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from frompitoangle import *
import os, time
import sys
from geometry_msgs.msg import WrenchStamped
class NetfData():

    def __init__(self, name = "netf_utlis_node" ):

        self.name = name
        # self.pos_dict = {}
        self.netf_buff_force = []
        self.netf_buff_torque = []

        self.ave_netf_force_data = []
        self.now_netf_torque_data = []
        self.now_netf_force_data = []
        self.ave_netf_torque_data = []
        self.tmp_sum_force= [0]*3
        self.tmp_sum_torque = [0] * 3

        # pass

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/netft_data", WrenchStamped, self.callback)
        return sub

    def callback(self, msg):
        self.read_force_from_netf( msg )
        self.read_torque_from_netf(msg)
        self.netf_buff_force, self.ave_netf_force_data = self.data_filter_netf_force( self.netf_buff_force, self.now_netf_force_data )
        self.netf_buff_torque, self.ave_netf_torque_data = self.data_filter_netf_torque(self.netf_buff_torque,self.now_netf_torque_data)

    def read_force_from_netf(self, msg):
        self.now_netf_force_data = [msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]
        #print msg.wrench
    def read_torque_from_netf(self, msg):
        self.now_netf_torque_data = [msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z]
        #print msg.wrench
    def data_filter_netf_force(self, pos_buff, new_data ):
        ave_netf_data = [0]*3
        if len( pos_buff ) == 10 :
            # print("new_data:", new_data)
            # print("tmp_sum before:", tmp_sum)
            # print("pos_buff[0]:", pos_buff[0])
            res1 = list_element_minus( self.tmp_sum_force , pos_buff[0] )
            self.tmp_sum_force = list_element_plus( res1 , new_data )
            # print("----------res1:", res1)
            # print("----------tmp_sum after:", tmp_sum)
            pos_buff = pos_buff[1:]
            pos_buff.append(new_data)
            ave_netf_data = list_element_multiple( self.tmp_sum_force, 1.0/10 )
            # print( "len:", len( pos_buff ))
            # print ("10----ave_pos_ur:", ave_ur_pose)
            # time.sleep(2)
            # sys.exit(0)
        else:
            pos_buff.append(new_data)
            self.tmp_sum_force = list_element_plus( self.tmp_sum_force, new_data )
            ave_netf_data = pos_buff[-1] # get the last element
            # print ("----------tmp_sum:", self.tmp_sum)
        # pos_buff.append( new_data )
        # print("---------------len:", pos_buff)
        return pos_buff, ave_netf_data

    def data_filter_netf_torque(self, pos_buff, new_data ):
        ave_netf_data = [0]*3
        if len( pos_buff ) == 10 :
            # print("new_data:", new_data)
            # print("tmp_sum before:", tmp_sum)
            # print("pos_buff[0]:", pos_buff[0])
            res1 = list_element_minus( self.tmp_sum_torque , pos_buff[0] )
            self.tmp_sum_torque = list_element_plus( res1 , new_data )
            # print("----------res1:", res1)
            # print("----------tmp_sum after:", tmp_sum)
            pos_buff = pos_buff[1:]
            pos_buff.append(new_data)
            ave_netf_data = list_element_multiple( self.tmp_sum_torque, 1.0/10 )
            # print( "len:", len( pos_buff ))
            # print ("10----ave_pos_ur:", ave_ur_pose)
            # time.sleep(2)
            # sys.exit(0)
        else:
            pos_buff.append(new_data)
            self.tmp_sum_torque = list_element_plus( self.tmp_sum_torque, new_data )
            ave_netf_data = pos_buff[-1] # get the last element
            # print ("----------tmp_sum:", self.tmp_sum)
        # pos_buff.append( new_data )
        # print("---------------len:", pos_buff)
        return pos_buff, ave_netf_data



def list_element_plus( v1, v2):
    res = list(map( lambda x: x[0] + x[1] , zip(v1,v2)))
    # print "plus :", res
    return res

def list_element_minus( v1, v2):
    res = list(map( lambda x: x[0] - x[1] , zip(v1,v2)))
    # print "minus :", res
    return res

def list_element_multiple( v1, num ):
    return [ item * num  for item in v1 ]


def main():
    netf_info_reader = NetfData()
    netf_info_reader.Init_node()
    while not rospy.is_shutdown():


        print ("ave_netf_force_data:", netf_info_reader.ave_netf_force_data)
        print ("now_netf_force_data: ", netf_info_reader.now_netf_force_data)


        print("ave_netf_torque_data",netf_info_reader.ave_netf_torque_data)
        print ("now_netf_torque_data: ", netf_info_reader.now_netf_torque_data)



if __name__ == "__main__":

    main()