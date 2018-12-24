#!usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
#from hand_in_eye import *


def tr2jac_new(T,samebody):
    # T = np.array(T)
    R = tr2r(T)
    # jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody == 1:
        jac_part1 = R
        New_trans = np.dot(-1 * (R.I), transl(T))
        jac_part2 = -np.dot(R, skew(New_trans))
        print "self.transl(T))", transl(T)
        # T1=[1,2,3]
        print "self.skew(self.transl(T))\n", skew(New_trans)
        jac_part3 = np.zeros((3, 3))
        jac_part4 = R

    else:
        jac_part1 = R
        jac_part2 = np.zeros((3, 3))
        jac_part3 = np.zeros((3, 3))
        jac_part4 = R
    jac_row1 = np.column_stack((jac_part1, jac_part2))
    jac_row2 = np.column_stack((jac_part3, jac_part4))
    jac = np.row_stack((jac_row1, jac_row2))
    return jac
def tr2jac( T, samebody):
    #T = np.array(T)
    R = tr2r(T)
    #jac = np.zeros((6, 6))
    """
    jac = [ jac_part1,  jac_part2;
            jac_part3,  jac_part4;
                ]
    """
    if samebody==1:
        jac_part1 = R.T
        jac_part2 = -np.dot( R.T,skew( transl(T)))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T

    else:
        jac_part1 = R.T
        jac_part2 = np.zeros((3,3))
        jac_part3 = np.zeros((3,3))
        jac_part4 = R.T
    jac_row1 = np.column_stack( (jac_part1, jac_part2) )
    jac_row2 = np.column_stack( (jac_part3, jac_part4) )
    jac = np.row_stack( (jac_row1, jac_row2) )
    return jac

"""
if l is 3*1 , then get 
skew(l) = [ 0, -l(2), l(1)
            l(2), 0 , -l(0)
            -l(1), l(0), 0]
if l is 1*1, then get
skew(l) = [ 0 , -l[0]
            l[0], 0 ]

"""
def skew(l):
    a, b = np.shape(l)
    try:
        if a == 3:
            s = np.array( [ 0, -l[2], l[1], l[2], 0, -l[0], -l[1], l[0], 0 ] )
            s = s.reshape((3,3))
            #print "s:", s
            return s
        elif a == 1:
            s = np.array( [ 0, -l[0], l[0], 0])
            s = s.reshape( (2,2) )
            return s
    except:
        print("erro l size!!!  3*1 or 1*1 required!")


def tr2r(T):
    r = [ 0, 1, 2]
    c = [ 0, 1, 2]
    R1 = T[r]
    R = R1[:,c]
    return R

def transl(T):
    r = [3]
    c = [0 , 1, 2]
    l1 = T[:, r]
    l = l1[c]
    return l

def test_main():
    pass
    # 1, get the X matrix
    # X = get_ur3_X()
    # print "rotation:", tr2r(X)
    # print "transition:", transl(X)
    # jac = tr2jac(X)
    # print "jac:", jac

    # a, b =np.shape(X[:,3])
    # print X[:,3]
    # print a,b

    # 2, get  the  samebody transfer matrix to jacobian matrix
    #jac = tr2jac(X)
    #print "jac", jac


if __name__=="__main__":
    test_main()