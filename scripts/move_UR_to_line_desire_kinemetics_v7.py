#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy
from std_msgs.msg import String,Float64
from frompitoangle import *
from ur5_kinematics import *
from ur5_pose_get import *
from transfer import *
from trans_methods import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from ur_tool_velocity_sub import *
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from pykdl_utils.kdl_kinematics import KDLKinematics

class UrLineCircle:
    def __init__(self,weights,radius,urdfname,p,Kp):
        self.weights = weights
        self.radius=radius#m
        self.vel = 0.1
        self.ace = 50
        self.cont=50
        self.t=0
        self.theta=-math.pi / 4
        self.tempq=[]
        self.p=p
        self.Kp=Kp
        self.urdfname=urdfname
        # rotating 45 degree with Z aisx
        self.wRb = [math.cos(self.theta), -1*math.sin(self.theta), 0, math.sin(self.theta), math.cos(self.theta), 0, 0, 0,1]
        self.uree_world_d_x_pub = rospy.Publisher("/ur_world_frame_ee_d_x", Float64, queue_size=10)
        self.uree_world_d_y_pub = rospy.Publisher("/ur_world_frame_ee_d_y", Float64, queue_size=10)
        self.uree_world_d_z_pub = rospy.Publisher("/ur_world_frame_ee_d_z", Float64, queue_size=10)
        self.uree_world_n_x_pub = rospy.Publisher("/ur_world_frame_ee_n_x", Float64, queue_size=10)
        self.uree_world_n_y_pub = rospy.Publisher("/ur_world_frame_ee_n_y", Float64, queue_size=10)
        self.uree_world_n_z_pub = rospy.Publisher("/ur_world_frame_ee_n_z", Float64, queue_size=10)
        self.uree_world_v_x_pub = rospy.Publisher("/ur_world_frame_ee_v_x", Float64, queue_size=10)
        self.uree_world_v_y_pub = rospy.Publisher("/ur_world_frame_ee_v_y", Float64, queue_size=10)
        self.uree_world_v_z_pub = rospy.Publisher("/ur_world_frame_ee_v_z", Float64, queue_size=10)
    def Init_node(self):
        rospy.init_node("move_ur5_circle")
        pub = rospy.Publisher("/ur_driver/URScript", String, queue_size=10)
        return pub
    def get_urobject_ur5kinetmatics(self):
        ur0 = Kinematic()
        return ur0
    def get_jacabian_from_joint(self,jointq):
        #robot = URDF.from_xml_file("/data/ros/ur_ws/src/universal_robot/ur_description/urdf/ur5.urdf")
        robot = URDF.from_xml_file(self.urdfname)
        tree = kdl_tree_from_urdf_model(robot)
        # print tree.getNrOfSegments()
        chain = tree.getChain("base_link", "ee_link")
        # print chain.getNrOfJoints()
        # forwawrd kinematics
        kdl_kin = KDLKinematics(robot, "base_link", "ee_link")
        q=jointq
        #q = [0, 0, 1, 0, 1, 0]
        pose = kdl_kin.forward(q)  # forward kinematics (returns homogeneous 4x4 matrix)
        # print pose
        #print list(pose)
        q0=Kinematic()
        # if flag==1:
        #     q_ik=q0.best_sol_for_other_py( [1.] * 6, 0, q0.Forward(q))
        # else:
        q_ik = kdl_kin.inverse(pose)  # inverse kinematics
        # print "----------iverse-------------------\n", q_ik

        if q_ik is not None:
            pose_sol = kdl_kin.forward(q_ik)  # should equal pose
            print "------------------forward ------------------\n",pose_sol

        J = kdl_kin.jacobian(q)
        #print 'J:', J
        return J,pose

    def caculate_vlocity_by_jocabian(self,t,Tstar,q_joint_t):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "Jacabian_t",Jacabian_singularity
        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        qdot_t=numpy.dot(Jacabian_singularity.I,tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T)
        print "qdot_t",self.Kp*t*qdot_t.T
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*t*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def caculate_vlocity_by_jocabian_pinv(self,t,Tstar,q_joint_t):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "Jacabian_plus",Jacabian_plus.reshape(6,6)
        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        qdot_t=numpy.dot(Jacabian_plus.I,tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T)
        print "qdot_t",self.Kp*t*qdot_t.T
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*1*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def caculate_vlocity_by_jocabian_pinv_new(self,cn,Tstar,q_joint_t,deltax):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "real time pose x",(pose[0].tolist()[0][3])
        print "Tstar",Tstar[3],Tstar[3]-(pose[0].tolist()[0][3])
        print "Jacabian_plus",Jacabian_plus.reshape(6,6)
        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T
        new_deta_v=numpy.array([[deltax,0,0,0,0,0]])
        qdot_t=numpy.dot(Jacabian_plus.I,new_deta_v.T)

        print "qdot_t",self.Kp*1*qdot_t.T
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*cn*qdot_t.T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def caculate_vlocity_by_jocabian_pinv_new_new(self,cn,Tstar,q_joint_t,deltax):
        """

        :return:
        """
        Jacabian_t,pose=self.get_jacabian_from_joint(q_joint_t)
        Jacabian_plus=numpy.dot(numpy.dot(Jacabian_t.T,Jacabian_t).I,Jacabian_t.T)
        # Jacabian_singularity=numpy.matrix(Jacabian_t).reshape(6,6)+self.p*numpy.eye(6)
        print "Jacabian_t",Jacabian_t
        print "real time pose x",(pose[0].tolist()[0][3])
        print "Tstar",Tstar[3],Tstar[3]-(pose[0].tolist()[0][3])
        print "Jacabian_plus",Jacabian_plus.reshape(6,6)

        print "tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T",tr2delta(numpy.array(Tstar).reshape(4,4),numpy.array(pose).reshape(4,4)).T

        new_jacabian_t=Jacabian_t[:5,:5]
        new_deta_v=numpy.array([[deltax,deltax,0,0,0]])

        qdot_t=numpy.dot(new_jacabian_t.I,new_deta_v.T).tolist()
        print "qdot_t",qdot_t
        new_qdot_t=[qdot_t[0][0],qdot_t[1][0],qdot_t[2][0],qdot_t[3][0],qdot_t[4][0],0]
        print "qdot_t",new_qdot_t
        print "numpy.array(q_joint_t).T",numpy.array(q_joint_t).T
        new_q_t_1=numpy.array(q_joint_t).T+self.Kp*cn*numpy.array(new_qdot_t).T

        print "new_q_t1",new_q_t_1
        return new_q_t_1
    def get_draw_circle_xy(self,t,xy_center_pos):
        x = xy_center_pos[0] + self.radius * math.cos( 2 * math.pi * t / self.cont )
        y = xy_center_pos[1] + self.radius * math.sin( 2 * math.pi * t / self.cont)
        return  [x,y]
    def get_draw_line_x(self,t,transxyz0,transxyz_d):#transxyz[x,y,z]
        xn_1=1*(transxyz_d[0]-transxyz0[0])/self.cont
        return xn_1

    def get_IK_from_T(self,T,q_last):
        ur0 = self.get_urobject_ur5kinetmatics()
        return ur0.best_sol(self.weights,q_last,T)

    def get_T_translation(self, T):
        trans_x = T[3]
        trans_y = T[7]
        trans_z = T[11]
        return [trans_x, trans_y, trans_z]
    def insert_new_xy(self,T,nx,ny,nz):
        temp=[]
        for i in xrange(16):
            if i==3:
                temp.append(nx)
            elif i==7:
                temp.append(ny)
            elif i == 11:
                temp.append(nz)
            else:
                temp.append(T[i])
        return temp
    def numpyarray_tolist(self,T):
        tt=T.tolist()
        temp=[]
        for i in range(4):
            for j in range(4):
                temp.append(tt[i][j])
        return temp

    def urscript_pub(self, pub, qq, vel, ace, t):

        ss = "movej([" + str(qq[0]) + "," + str(qq[1]) + "," + str(qq[2]) + "," + str(
            qq[3]) + "," + str(qq[4]) + "," + str(qq[5]) + "]," + "a=" + str(ace) + "," + "v=" + str(
            vel) + "," + "t=" + str(t) + ")"
        print("---------ss:", ss)
            # ss="movej([-0.09577000000000001, -1.7111255555555556, 0.7485411111111111, 0.9948566666666667, 1.330836666666667, 2.3684322222222223], a=1.0, v=1.0,t=5)"
        pub.publish(ss)

    """
        [RwbRbe Rwbtbe
        0          1  ]
        """

    def caculate_world_frame(self, T):
        bRe = tr2r(T)
        bte = transl(T)
        wRb = numpy.array(self.wRb).reshape((3, 3))
        homegeneous_T_part3 = numpy.array([0, 0, 0, 1])
        wRbbRe = numpy.dot(wRb, bRe)
        wRbbte = numpy.dot(wRb, bte)
        new_T_T = numpy.column_stack((wRbbRe, wRbbte))
        new_T = np.row_stack((new_T_T, homegeneous_T_part3))
        last_T = self.numpyarray_tolist(new_T)
        inv_wTb_part = numpy.array([0, 0, 0]).T
        inv_wTb_1 = numpy.column_stack((wRb, inv_wTb_part))
        inv_wTb_2 = np.row_stack((inv_wTb_1, homegeneous_T_part3))
        return last_T, inv_wTb_2
    def move_ee_to_desire_point_in_world_frame(self,ur_pub,ur0,Zero_T,world_frame_nx,world_frame_ny,world_frame_nz):
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(Zero_T).reshape((4, 4)))
        new_T_in_world_frame=self.insert_new_xy(WTe,world_frame_nx,world_frame_ny,world_frame_nz)#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        q_new = ur0.best_sol(self.weights, self.tempq, new_T_list)
        print "q_new"
        self.urscript_pub(ur_pub, q_new, self.vel, self.ace, self.t)

    def move_ee_to_left(self,ur5_pub,uro_kinematicx,Qstart,world_frame_nx):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3]+world_frame_nx,WTe[7],WTe[11])#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        self.uree_world_d_x_pub.publish(WTe[3]+world_frame_nx)
        self.uree_world_d_y_pub.publish(WTe[7])
        self.uree_world_d_z_pub.publish(WTe[11])

        q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)

        new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new
    def move_ee_to_right(self,ur5_pub,uro_kinematicx,Qstart,world_frame_nx,cn):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        self.uree_world_d_x_pub.publish(WTe[3] - world_frame_nx)
        self.uree_world_d_y_pub.publish(WTe[7])
        self.uree_world_d_z_pub.publish(WTe[11])

        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3]-world_frame_nx,WTe[7],WTe[11])#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))

        # new_T_list=self.numpyarray_tolist()
        new_T_list=self.insert_new_xy(F_T,F_T[3]-world_frame_nx,F_T[7],F_T[11])
        # q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)
        #impedance control for jacabian
        # q_new_from_jacabian=self.caculate_vlocity_by_jocabian(cn,new_T_list,Qstart).tolist()[0]

        q_new_from_jacabian=self.caculate_vlocity_by_jocabian_pinv_new_new(cn,new_T_list,Qstart,world_frame_nx).tolist()
        q_new=q_new_from_jacabian
        print "q_new_from_jacabian",q_new_from_jacabian
        # new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        # print "real new T",numpy.array(F_T).reshape(4,4)
        # print "new_T_inbase_after_caculate",numpy.array(new_T_inbase_after_caculate).reshape(4,4)
        # new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        # self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        # self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        # self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new
    def move_ee_to_right_new(self,ur5_pub,uro_kinematicx,Qstart,world_frame_nx):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        self.uree_world_d_x_pub.publish(WTe[3] - world_frame_nx)
        self.uree_world_d_y_pub.publish(WTe[7])
        self.uree_world_d_z_pub.publish(WTe[11])
        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3]-world_frame_nx,WTe[7],WTe[11])#0.55
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)

        q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)
        new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new
    def move_ee_to_up(self):
        pass
    def move_ee_to_down(self,ur5_pub,uro_kinematicx,Qstart,world_frame_ny):
        F_T = uro_kinematicx.Forward(Qstart)
        WTe, inv_wTb = self.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
        print "WTe",numpy.array(WTe).reshape((4,4))
        new_T_in_world_frame=self.insert_new_xy(WTe,WTe[3],WTe[7]-world_frame_ny,WTe[11])#0.55
        self.uree_world_d_x_pub.publish(WTe[3])
        self.uree_world_d_y_pub.publish(WTe[7]-world_frame_ny)
        self.uree_world_d_z_pub.publish(WTe[11])
        new_T_in_base_frame=numpy.dot(numpy.matrix(inv_wTb).I,numpy.matrix(new_T_in_world_frame).reshape((4,4)))
        new_T_list=self.numpyarray_tolist(new_T_in_base_frame)
        q_new = uro_kinematicx.best_sol(self.weights, self.tempq, new_T_list)
        new_T_inbase_after_caculate=uro_kinematicx.Forward(q_new)
        new_T_in_world_after_caculate,inv_new_T=self.caculate_world_frame(numpy.array(new_T_inbase_after_caculate).reshape((4,4)))
        self.uree_world_n_x_pub.publish(new_T_in_world_after_caculate[3])
        self.uree_world_n_y_pub.publish(new_T_in_world_after_caculate[7])
        self.uree_world_n_z_pub.publish(new_T_in_world_after_caculate[11])
        self.urscript_pub(ur5_pub, q_new, self.vel, self.ace, self.t)
        return q_new

def main():
    t=0
    vel=0.1
    ace=50
    # vel=1.05
    # ace=1.4
    transxyz_d=[0.1,0,0]
    urdfname = "/data/ros/ur_ws_yue/src/ur5_planning/urdf/ur5.urdf"
    qstart=[-46.658908262958036, -174.89198611081196, 80.03454750164096, -168.48910996772918, -90.12196190665009, 182.75583607489475]

    ratet = 25
    radius=0.1
    weights = [1.] * 6
    T_list=[]
    p=0.001
    Kp=-1
    urc=UrLineCircle(weights,radius,urdfname,p,Kp)
    pub=urc.Init_node()
    rate = rospy.Rate(ratet)

    # first step go to initial pos
    qzero = display(getpi(qstart))
    # urc.urscript_pub(pub,q,vel,ace,t)
    # second get T use urkinematics
    urk = urc.get_urobject_ur5kinetmatics()
    F_T = urk.Forward(qzero)
    # ur5_pub=urc.
    # print "F_T", F_T
    TransT = urc.get_T_translation(F_T)

    ur_reader = Urposition()
    ur_sub = rospy.Subscriber("/joint_states", JointState, ur_reader.callback)

    tool0 = UrToolVelocityRead()
    tool_sub = rospy.Subscriber("/tool_velocity", TwistStamped, tool0.Ur_tool_velocity_callback)

    WTe, inv_wTb = urc.caculate_world_frame(numpy.array(F_T).reshape((4, 4)))
    ur0_kinematics=urc.get_urobject_ur5kinetmatics()
    cn=1
    ZeroT = F_T
    urc.tempq = qzero
    QTemp=[]
    flag_to_zero=0
    flag_to_right=1

    while not rospy.is_shutdown():

        if len(ur_reader.ave_ur_pose)!=0:
            q_now = ur_reader.ave_ur_pose
            if flag_to_right==1:
                print "first move to right -----"
                detax=urc.get_draw_line_x(cn,[0.286,0,0],[0.5,0,0])
                qq=urc.move_ee_to_right(pub,ur0_kinematics,q_now,detax,cn)
                print cn, "move to right -----", qq
                cn+=1
                time.sleep(0.1)
                q_now = ur_reader.ave_ur_pose

                if cn==(urc.cont-1):

                    flag_to_zero=1
                    flag_to_right=0
                    time.sleep(4)
                    cn=1

        rate.sleep()
if __name__ == '__main__':
        main()