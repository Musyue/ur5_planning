#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers

from frompitoangle import *
import os, time

TAGNUM = 1

class arReader():

    def __init__(self, name = "ar_info_subscriber" ):

        self.name = name
        # self.marklist = [None]*3
        self.pos_dict = {}
        self.ar_position_buff_dict = {}
        self.tmp_sum_dict = {}
        for i in range(TAGNUM):
            self.ar_position_buff_dict[i] = []
            self.tmp_sum_dict[i] = [0]*7
        # self.ar_position_buff = []
        self.ave_pos_dict = {}
        # self.tmp_sum_dict = {}

        # pass

    def Init_node(self):
        rospy.init_node(self.name)
        sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback)
        return sub

    def callback(self, msg):
        global TAGNUM
        ar_position = [ ]
        # ar_quaternion = [ ]
        # mark_list = []
        for i in range(TAGNUM):
            self.pos_dict[msg.markers[i].id ] = self.read_pos_from_ar_markers( msg, i )
            # print("pose------",self.pos_dict[msg.markers[i].id ])

        # buff size : 10

        for i in range(TAGNUM):
            self.ar_position_buff_dict[i], self.ave_pos_dict[i] = self.pos_filter_ar( self.ar_position_buff_dict[i] , self.pos_dict[i], i )



    def read_pos_from_ar_markers(self, msg, i):
        pos_msg = msg.markers[i].pose.pose.position
        quaternion_msg = msg.markers[i].pose.pose.orientation
        ar_position = [ pos_msg.x , pos_msg.y, pos_msg.z, quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w  ]
        return ar_position

    def pos_filter_ar(self, pos_buff, new_data, i ):
        # self.tmp_sum_dict[i] = [0]*10
        ave_ar_pose = [0]*7
        if len( pos_buff ) == 10 :
            # print("new_data:", new_data)
            # print("tmp_sum before:", tmp_sum)
            # print("pos_buff[0]:", pos_buff[0])
            res1 = list_element_minus( self.tmp_sum_dict[i] , pos_buff[0] )
            self.tmp_sum_dict[i] = list_element_plus( res1 , new_data )
            # print("res1:", res1)
            # print("tmp_sum after:", tmp_sum)
            pos_buff = pos_buff[1:]
            pos_buff.append(new_data)
            ave_ar_pose = list_element_multiple( self.tmp_sum_dict[i], 1.0/10 )
            # print( "len:", len( pos_buff ))
        else:
            pos_buff.append(new_data)
            self.tmp_sum_dict[i] = list_element_plus( self.tmp_sum_dict[i] , new_data )
            ave_ar_pose = pos_buff[-1] # get the last element

        # pos_buff.append( new_data )
        return pos_buff, ave_ar_pose



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
    ar_info_reader = arReader()
    ar_info_reader.Init_node()
    # print ("now_pos: ", ar_info_reader.pos_dict)
    # print ("ave_pos_ur:", ar_info_reader.ave_pos_dict)
    # rospy.spin()
    while not rospy.is_shutdown():
        # pass
        print ( "now_pos: ", ar_info_reader.pos_dict)

        print ("ave_pos_ur:", ar_info_reader.ave_pos_dict)


if __name__ == "__main__":
    # v1 = [1,2,43.0,5]
    # v2 = [3, 1, 0.9, 2.9 ]
    # print list_element_minus( v1 , v2 )
    main()