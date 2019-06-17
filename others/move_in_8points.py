#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
#for cartesian planning
from cartesian_planning import Cartesianplan
class TrajectoryTest():
    def __init__(self,arm_joints,listQn,sim):
        # 机械臂中joint的命名
        self.arm_joints = arm_joints
        # 是否需要回到初始化的位置
        #reset = rospy.get_param('~reset', False)
        # whether or not simluation
        self.sim = sim
        self.listQn=listQn
    def init_ur(self):
        rospy.init_node('trajectory_demo')
        # 连接机械臂轨迹规划的trajectory action server
        rospy.loginfo('Waiting for arm trajectory controller...')
        if self.sim:
            arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            rospy.loginfo('...connected.')
        else:
            arm_client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            arm_client.wait_for_server()
            rospy.loginfo('real ur5 ...connected.')
        return arm_client
    def set_trajectory(self):
        #from Q0-->Q7
        # 使用设置的目标位置创建一条轨迹数据
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_joints
        #Get joint_states in time
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
                arm_trajectory.points[i+1].positions = self.listQn[i]
                arm_trajectory.points[i+1].velocities = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i+1].accelerations = [0.0 for j in self.arm_joints]
                arm_trajectory.points[i+1].time_from_start = rospy.Duration(i+3)
        return arm_trajectory
    def move_ur5(self):
        arm_client=self.init_ur()
        arm_trajectory=self.set_trajectory()
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
    #lista=[[-0.25277, -0.8561733333333333, 1.2195411111111112, -3.557096666666667, -1.3205444444444445, -1.1349355555555556], [-0.0020933333333333333, -0.8746644444444445, 1.2195411111111112, -3.4805155555555563, -1.5616266666666665, -1.13511], [0.24893222222222225, -0.7698233333333334, 0.5587455555555556, -2.8842644444444447, -1.7582255555555557, -1.13511], [0.14374222222222224, -0.7140011111111112, 0.11356333333333334, -2.554041111111111, -1.7018800000000003, -1.13511], [-0.009594444444444445, -0.7153966666666667, 0.10937666666666668, -2.6421355555555555, -1.6348933333333333, -1.13511], [-0.2688188888888889, -0.732841111111111, 0.10972555555555556, -2.514791111111111, -1.2882722222222223, -1.13511], [-0.29376444444444444, -0.952641111111111, 0.9927633333333333, -3.250074444444445, -1.2884466666666667, -1.1352844444444443], [-0.05303111111111112, -0.9106000000000002, 0.8605344444444445, -3.1536066666666667, -1.471438888888889, -1.13511]]
    #lista=[[0,0,0,0,0,0],[6.0304153000000005, 5.323002462548293, 0.9029269675929017, 0.00511963446407448, 1.3205443796280261, 2.006657098420414], [6.0304153000000005, 6.187074219589875, 5.380258332407099, 0.9469018197878816, 1.3205443796280261, 2.006657098420414], [6.0304153000000005, 5.427011964466014, 1.219541115006123, 2.726088640500936, 4.9626409203719746, 5.148249744830621], [6.0304153000000005, 0.30739363204291714, 5.063644184993877, 4.001603902936279, 4.9626409203719746, 5.148249744830621], [3.2265509455389454, 2.8339774682112884, 1.2201075127315175, 5.421395861475703, 1.4837578408833672, 5.130916030202064], [3.2265509455389454, 3.99807707201484, 5.063077787268483, 0.41432598313518487, 1.4837578408833672, 5.130916030202064], [3.2265509455389454, 3.2379411467344807, 0.902249352569037, 2.1936976889010857, 4.7994274591166333, 1.989323383791857], [3.2265509455389454, 4.101368808680123, 5.380935947430963, 3.1347687392731043, 4.7994274591166333, 1.989323383791857]]
    #lista=[[0,0,1,0,1,0],[6.0304153000000005, 5.323002462548293, 0.9029269675929017, 0.00511963446407448, 1.3205443796280261, 2.006657098420414]]
    #lista=[[0,0,1,0,0,1],[-0.25277, -0.8561733333333333, 1.2195411111111112, -3.557096666666667, -1.3205444444444445, -1.1349355555555556]]
    #print lista
    #lista=[[-0.25277000717958575, -0.8561733427135723, 1.219541115006123, -3.55709666667865, -1.3205443868076117, -1.1349355623489652], [-0.22543604295828334, -0.8611933576518771, 1.2261321805493186, -3.5583253613116197, -1.3478414589876122, -1.136395977625826], [-0.19784003717321497, -0.8650043975466071, 1.2305831725767058, -3.558662901506949, -1.3754006795731515, -1.1378521720523622], [-0.17002251387309375, -0.8676058726247131, 1.2329045167506603, -3.558120663454388, -1.403181526442694, -1.1393040677124766], [-0.1420257489755956, -0.8689959378108503, 1.2331016093019842, -3.556706151592764, -1.4311417251943093, -1.1407516030025722], [-0.11389339336366078, -0.8691716608745086, 1.231175035689761, -3.5544230468440143, -1.459237626037842, -1.1421947330891111], [-0.08567006746089945, -0.8681290709492613, 1.2271205891422583, -3.5512711755540924, -1.4874246092130576, -1.1436334301648658], [-0.057400936623608345, -0.865863086714663, 1.2209290899519638, -3.547246400748519, -1.5156575095963758, -1.1450676834983975], [-0.02913127742689081, -0.8623673203638722, 1.2125859979054214, -3.5423404319510086, -1.5438910504187282, -1.14649749927705]]
    #lista=[[-0.25277000717958575, -0.8561733427135723, 1.219541115006123, -3.55709666667865, -1.3205443868076117, -1.1349355623489652],[0.24893222222222225, -0.7698233333333334, 0.5587455555555556, -2.8842644444444447, -1.7582255555555557, -1.13511]]
    lista=[[3.3985266666666667, -1.3765411111111112, -1.944881111111111, 0.14112555555555556, -4.9428833333333335, 5.663687777777778],[2.114018604707434, -2.3149859000384527, -0.8966068077429779, 0.10966440472194561, -3.654160624143393, 5.528734732653671]]
    arm_joints = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    sim = rospy.get_param('~sim', True)
    try:
        node0 = TrajectoryTest(arm_joints,lista,sim)
        node0.init_ur()
        node0.set_trajectory()
        node0.move_ur5()
    except rospy.ROSInterruptException:
        print lista


if __name__ == '__main__':
    main()
