#include <ros/ros.h>
#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <ur5_planning/ur_kin.h>
#include <geometry_msgs/WrenchStamped.h>
#include <ur5_planning/forcecontrol_msg.h>


const double PI = M_PI;

double q_cur[6] = {0.0,-PI,PI/2,-PI/2,PI/2,0.0};

double f_val_from_sensor[3]={0.0,0.0,0.0};
double f_val_record[3]={0.0,0.0,0.0};


void get_force_value_from_force_sensor(const std_msgs::UInt16& msg)
{
    f_val_from_sensor[0]=0.0;
    f_val_from_sensor[1]=0.0;
    f_val_from_sensor[2]=0.0;
}

#define FILTER_A 0.5
int Value=0;
int NewValue=0;
int first_order_filtering(int input_value)
{
    NewValue = input_value;
    Value=(int)((float)NewValue * FILTER_A + (1.0 - FILTER_A) * (float)Value);
    return Value;
}

void callback(const sensor_msgs::JointState& msg)
{
    q_cur[0]=msg.position[0];
    q_cur[1]=msg.position[1];
    q_cur[2]=msg.position[2];
    q_cur[3]=msg.position[3];
    q_cur[4]=msg.position[4];
    q_cur[5]=msg.position[5];
}

void get_force_from_ati(const geometry_msgs::WrenchStamped& msg)
{
    f_val_from_sensor[0]=msg.wrench.force.z;
    f_val_from_sensor[1]=0.0;
    f_val_from_sensor[2]=0.0;
    f_val_record[0]=msg.wrench.force.x;
    f_val_record[1]=msg.wrench.force.y;
    f_val_record[2]=msg.wrench.force.z;    
}
void get_new_T_after_translation(double T[], double translation[]);
int main( int argc, char** argv )
{
  double q[6],last_q[6];
  double x,y,z;
  double x_init,y_init,z_init;;
  double T[16], T_init[16];
  double translation[3],tran[3],rot[9];
  double t=0;
  double R=0.05;
  double force_error;
  int id=1;

  ros::init( argc, argv, "ur_forcecontrol_v1" );
  ros::NodeHandle nh;
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>( "/joint_states", 1 );
  ros::Publisher script_pub = nh.advertise<std_msgs::String>("ur_driver/URScript", 10);
  ros::Subscriber joint_sub=nh.subscribe("/joint_states",1,callback);
  //ros::Subscriber force_sub=nh.subscribe("/adc",20,get_force_value_from_force_sensor);
  ros::Subscriber force_sub = nh.subscribe("netft_data", 100, get_force_from_ati);
  ros::Publisher  force_pub=nh.advertise<std_msgs::Float64>("/netft_filter",100);
  ros::Publisher record_pub=nh.advertise<ur5_planning::forcecontrol_msg>("/Record_Data",1);

  ur5_planning::forcecontrol_msg record_data;
  tf::TransformListener listener;
  int loop_rate = 30;
  ros::Rate rate( loop_rate );
  sensor_msgs::JointState joint_state;
  std_msgs::Float64 adc_filter_result;

  double force_val_desire[3];
  double xy_tran[3],xy_tran_base[3],force_tran_base[3];
//  double kp=0.000001, kd=0.0, k_spring=500;
double kp=0.00001, kd=0.0, k_spring=500;
//double kp=0.000001, kd=0.0, k_spring=500;
//double kp=0.00001, kd=0.0, k_spring=500;

//  force_val_desire[0]=100;
//  force_val_desire[1]=0.0;
//  force_val_desire[2]=0.0;

    force_val_desire[0]=0.0;
    force_val_desire[1]=0.0;
    force_val_desire[2]=0.0;

  for(int j=0;j<20;j++)
  {
      for (int i = 0; i < 6; ++ i) {
          q[i] = q_cur[i];
      }
      ros::spinOnce();
      rate.sleep();
  }

  ur_kinematics::solve("fk", q_cur, T_init, NULL,5);
  x_init=T_init[3];
  y_init=T_init[7];
  z_init=T_init[11];

  for (int i = 0; i < 6; ++ i) {
      last_q[i] = q[i];
  }
  x=x_init;
  y=y_init;
  z=z_init;
  ROS_INFO("the begin point joint angles are: %lf, %lf, %lf, %lf, %lf, %lf",q[0],q[1],q[2],q[3],q[4],q[5]);

  while (ros::ok) {
      ROS_INFO("Initial position is: %lf, %lf,%lf",x_init,y_init,z_init);
      ROS_INFO("the coordinates are: %f, %f, %f.",x,y,z);

      if (ur_kinematics::solve("fk", q, T, NULL,5)) {
          ROS_INFO("solve forward kinematics");
      }else {
          ROS_INFO("Failed to call service ur5_kinematics in fk");
          return 1;
      }

      for(int m=0;m<3;m++)
      {
          tran[m]=T[4*m+3];
          for (int n=0;n<3;n++)
          {
              rot[3*m+n]=T[4*m+n];
          }
      }
      //f_val_from_sensor[0]=first_order_filtering(f_val_from_sensor[0]);
      //adc_filter_result.data=static_cast<unsigned int>(f_val_from_sensor[0]);
      adc_filter_result.data=f_val_from_sensor[0];
      force_pub.publish(adc_filter_result);
      ROS_INFO("The desire force value is %lf",force_val_desire[0]);
      ROS_INFO("The real force value is %lf",f_val_from_sensor[0]);

     
      if(t==0)
      {
          xy_tran[0]=0;
          xy_tran[1]=R*sin(t);
          xy_tran[2]=R-R*cos(t);
      }
      if(t>0)
      {
          xy_tran[0]=0;
          xy_tran[1]=R*sin(t)-R*sin(t-2*PI/200);
          xy_tran[2]=R*cos(t-2*PI/200)-R*cos(t);
      }
      
      /*
      xy_tran[0]=0;
      xy_tran[1]=0;
      xy_tran[2]=0;
       */      

      t=2*PI/200+t;
      if(t==2*PI+2*PI/200)
      {
          t=0;
      }
      force_error=fabs(force_val_desire[0]-f_val_from_sensor[0]);
      for (int k=0;k<3;k++)
      {
      force_tran_base[k]=-kp*(rot[3*k+0]*(force_val_desire[0]-f_val_from_sensor[0])+rot[3*k+1]*(force_val_desire[1]-f_val_from_sensor[1])+rot[3*k+2]*(force_val_desire[2]-f_val_from_sensor[2]));
      translation[k]=force_tran_base[k];
      }

      id=id+1;
      ROS_INFO("force error is: %lf, and the id is %d",force_error,id);
      ROS_INFO("force_tran_base is %lf,%lf,%lf",force_tran_base[0],force_tran_base[1],force_tran_base[2]);
      ROS_INFO("xy_tran_base is %lf,%lf,%lf",xy_tran_base[0],xy_tran_base[1],xy_tran_base[2]);
      ROS_INFO("translation is %lf,%lf,%lf",translation[0],translation[1],translation[2]);

      get_new_T_after_translation(T, translation);

      if (ur_kinematics::solve("ik", q, T, NULL)) {
          ROS_INFO("solve inverse kinematics");
          for (int k = 0; k < 6; ++ k) {
              if (fabs(q[k]-PI*2.0-last_q[k]) < fabs(q[k]-last_q[k])) {
                  q[k] -= PI*2.0;
              }
          }
      }else {
          ROS_INFO("Failed to call service ur5_kinematics in ik");
          for (int i = 0; i < 6; ++ i) {
              q[i] = last_q[i];
          }
      }

      joint_state.name.resize(6);
      joint_state.position.resize(6);
      joint_state.name[0] = "shoulder_pan_joint";
      joint_state.name[1] = "shoulder_lift_joint";
      joint_state.name[2] = "elbow_joint";
      joint_state.name[3] = "wrist_1_joint";
      joint_state.name[4] = "wrist_2_joint";
      joint_state.name[5] = "wrist_3_joint";
      joint_state.header.stamp = ros::Time::now();
      for (int k = 0; k < 6; ++ k) {
          joint_state.position[k] = q[k];
      }
    ROS_INFO("the end point joint angles are: %lf, %lf, %lf, %lf, %lf, %lf",q[0],q[1],q[2],q[3],q[4],q[5]);
    //joint_pub.publish(joint_state);

    char urscript[1000];
    // servoj(q, a, v, t=0.008, lookahead time=0.1, gain=300)
    //sprintf(urscript, "servoj([%lf, %lf, %lf, %lf, %lf, %lf],50,0.1)",q[0], q[1], q[2], q[3], q[4], q[5]);
    sprintf(urscript, "movej([%lf, %lf, %lf, %lf, %lf, %lf], 50, 0.1)", q[0], q[1], q[2], q[3], q[4], q[5]);
    //sprintf(urscript, "speedj([%lf, %lf, %lf, %lf, %lf, %lf],50,%lf)",q_dot[0],q_dot[1],q_dot[2],q_dot[3],q_dot[4],q_dot[5],1.0/loop_rate);
    std_msgs::String msg;
    std::stringstream ss;
    ss<<urscript;
    msg.data=ss.str()+"\n";
    script_pub.publish(msg);

    for (int i = 0; i < 6; ++ i) {
        last_q[i] = q[i];
    }
    try{
      std::string destination_frame( "base_link" );
      std::string original_frame( "ee_link" );
      tf::StampedTransform transform;

      listener.lookupTransform(  destination_frame, original_frame, ros::Time(0), transform );
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      z = transform.getOrigin().z();
    }
    catch(tf::TransformException ex)
      { std::cout << ex.what() << std::endl; }

    x=T[3];
    y=T[7];
    z=T[11];

    record_data.x_error=rot[0]*(x-x_init)+rot[3]*(y-y_init)+rot[6]*(z-z_init);
    record_data.y_error=rot[1]*(x-x_init)+rot[4]*(y-y_init)+rot[7]*(z-z_init);
    record_data.z_error=rot[2]*(x-x_init)+rot[5]*(y-y_init)+rot[8]*(z-z_init);

    record_data.x=rot[0]*(x)+rot[3]*(y)+rot[6]*(z);
    record_data.y=rot[1]*(x)+rot[4]*(y)+rot[7]*(z);
    record_data.z=rot[2]*(x)+rot[5]*(y)+rot[8]*(z);

    record_data.x_init=rot[0]*(x_init)+rot[3]*(y_init)+rot[6]*(z_init);
    record_data.y_init=rot[1]*(x_init)+rot[4]*(y_init)+rot[7]*(z_init);
    record_data.z_init=rot[2]*(x_init)+rot[5]*(y_init)+rot[8]*(z_init);

    record_data.stamp=ros::Time::now();
    record_data.x_force=f_val_record[0];
    record_data.y_force=f_val_record[1];
    record_data.z_force=f_val_record[2];
    record_pub.publish(record_data);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

void get_new_T_after_translation(double T[], double translation[])
{
    T[3]  += translation[0];
    T[7]  += translation[1];
    T[11] += translation[2];
}

