#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
# for cartesian planning
from cartesian_planning import Cartesianplan


class TrajectoryTest():
    def __init__(self, arm_joints, listQn, sim):
        # 机械臂中joint的命名
        self.arm_joints = arm_joints
        # 是否需要回到初始化的位置
        # reset = rospy.get_param('~reset', False)
        # whether or not simluation
        self.sim = sim
        self.listQn = listQn

    def init_ur(self):
        rospy.init_node('trajectory_demo')
        # 连接机械臂轨迹规划的trajectory action server
        rospy.loginfo('Waiting for arm trajectory controller...')
        if self.sim:
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory',
                                                      FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            rospy.loginfo('...connected.')
        else:
            arm_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            rospy.loginfo('real ur5 ...connected.')
        return arm_client

    def set_trajectory(self):
        # from Q0-->Q7
        # 使用设置的目标位置创建一条轨迹数据
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_joints
        # Get joint_states in time
        if self.sim:
            for i in range(len(self.listQn)):
                arm_trajectory.points.append(JointTrajectoryPoint())
                arm_trajectory.points[i].positions = self.listQn[i]
                arm_trajectory.points[i].velocities = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i].accelerations = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i].time_from_start = rospy.Duration(i + 3)
        else:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            arm_trajectory.points.append(JointTrajectoryPoint())
            arm_trajectory.points[0].positions = joints_pos
            arm_trajectory.points[0].velocities = [0.0 for i in self.arm_joints]
            arm_trajectory.points[0].accelerations = [0.0 for i in self.arm_joints]
            arm_trajectory.points[0].time_from_start = rospy.Duration(0.0)
            for i in range(len(self.listQn)):
                arm_trajectory.points.append(JointTrajectoryPoint())
                arm_trajectory.points[i + 1].positions = self.listQn[i]
                arm_trajectory.points[i + 1].velocities = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i + 1].accelerations = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i + 1].time_from_start = rospy.Duration(i + 3)
        return arm_trajectory

    def move_ur5(self):
        arm_client = self.init_ur()
        arm_trajectory = self.set_trajectory()
        rospy.loginfo('Moving the arm to goal position...')
        # 创建一个轨迹目标的空对象
        arm_goal = FollowJointTrajectoryGoal()
        # 将之前创建好的轨迹数加入轨迹目标对象中
        arm_goal.trajectory = arm_trajectory
        # 设置执行时间的允许误差值
        arm_goal.goal_time_tolerance = rospy.Duration(1.0)
        # 将轨迹目标发送到action server进行处理，实现机械臂的运动控制
        arm_client.send_goal(arm_goal)
        # 等待机械臂运动结束
        arm_client.wait_for_result(rospy.Duration(5.0))
        rospy.loginfo('...done')


def main():
    c0=Cartesianplan()
    #q0=[-0.25277, -0.8561733333333333, 1.2195411111111112, -3.557096666666667, -1.3205444444444445, -1.1349355555555556]
    #q1=[-0.0020933333333333333, -0.8746644444444445, 1.2195411111111112, -3.4805155555555563, -1.5616266666666665, -1.13511]
    q0 = [-1.5653986853573416, -1.6473636790059931, 0.050482702000686414, -1.5414289490128499, -1.5287390249613757, -0.08883984525927555]
    #q1 = [-1.5653986853573416, -1.6473636790059931, 0.050482702000686414, -1.4414289490128499, -1.287390249613757, -0.08883984525927555]
    q1=[-0.0020933333333333333, -0.8746644444444445, 1.2195411111111112, -3.4805155555555563, -1.5616266666666665, -1.13511]
    #q1=[0.9997741309734645, -3.1415978002537317, -2.718607921115275, -0.8873630134189732, -0.37554362661911433, -0.09702105288460494]
    lista = c0.get_planning(q0,q1,0.1)

    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    sim = rospy.get_param('~sim',True)
    try:
        node0 = TrajectoryTest(arm_joints, lista, sim)
        node0.init_ur()
        node0.set_trajectory()
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        print joints_pos
        node0.move_ur5()
    except rospy.ROSInterruptException:
        print lista


if __name__ == '__main__':
    main()
