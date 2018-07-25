#!/usr/bin/env python
# -*- coding: utf-8 -*-
from new_kinematics import Kinematic
import math
#q:joint state angle
class Cartesianplan():
    def __init__(self):
        pass
       # self.q=q
    #d=v*t
    #L=sqrt()
    def get_displace(self,T0,Tf,d):
        result=[]
        L=math.sqrt((Tf[3]-T0[3])**2+(Tf[7]-T0[7])**2+(Tf[11]-T0[11])**2)
        #print "L-----------------",L
        N=int(L/d+1)
        detax=(Tf[3]-T0[3])/N
        result.append(detax)
        detay=(Tf[7]-T0[7])/N
        result.append(detay)
        detaz=(Tf[11]-T0[11])/N
        result.append(detaz)
        return result,N
    def add_deta(self,T,detax,detay,detaz,n):
        result=[]
        for i in range(len(T)):
            if i==3:
                result.append(T[3]+n*detax)
            elif i==7:
                result.append(T[7] + n*detay)
            elif i==11:
                result.append(T[11] + n*detaz)
            else:
                result.append(T[i])
        return result
    #just change displace vector
    def getdata_linear_with_no_pose_change(self,Tf,T0,d):
        deta,N=self.get_displace(T0,Tf,d)
        result=[]
        print deta,N
        for i in range(N):
            result.append(self.add_deta(T0,deta[0],deta[1],deta[2],i))
        return result
    def get_planning(self,q1,q2,d):
        p1=Kinematic(q1)
        p2=Kinematic(q2)
        listtemp = [0, 0, 0, 0, 0]
        p0 = Kinematic(listtemp)
        T1 = p1.Forward()
        T2 = p2.Forward()
        result=[]
        temp=self.getdata_linear_with_no_pose_change(T2,T1,d)
        for i in range(len(temp)):
            print "waypoints:",i+1
            result.append(p0.best_sol_for_other_py([1.] * 6, q1, temp[i]))
        print "----------------------------last result-----------------"
        print result
        return result
    def getdata_linear(self):
        pass
    def getdata_circle(self):
        pass
def main():
    #lista = [[-0.25277, -0.8561733333333333, 1.2195411111111112, -3.557096666666667, -1.3205444444444445, -1.1349355555555556],[0.24893222222222225, -0.7698233333333334, 0.5587455555555556, -2.8842644444444447, -1.7582255555555557, -1.13511]]
    lista=[[-0.012036666666666666, -1.7566555555555554, 1.7803800000000003, -3.0395200000000004, -1.572791111111111, -0.7911055555555556], [-0.012036666666666666, -1.5912822222222223, 1.0904522222222222, -2.636553333333333, -1.572791111111111, -0.79128]]
    p1=Kinematic(lista[0])
    p2=Kinematic(lista[1])
    listtemp=[0,0,0,0,0]
    p0=Kinematic(listtemp)
    T1=p1.Forward()
    print "T1--------------------->",T1
    T2=p2.Forward()
    print "T2-------------------->",T2
    c0=Cartesianplan()
    result=[]
    temp=c0.getdata_linear_with_no_pose_change(T2,T1,2)
    for i in range(len(temp)):
        result.append(p0.best_sol_for_other_py([1.] * 6,lista[0],temp[i]))
    print result
    print "*********************************"
    c0.get_planning(lista[0],lista[1],0.02)
if __name__ == '__main__':
    main()