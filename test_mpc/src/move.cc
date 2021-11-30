#include "move.h"
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>

#include <math.h>
#include "std_msgs/Int32.h"

#include <rosgraph_msgs/Clock.h>


using namespace Eigen;
using namespace ros;

int rti_num = 50;
MPC_solver myMpcSolver(rti_num);

void ref_trajectory_switch_cb(const std_msgs::Bool::ConstPtr& msg)
{
    ref_trajectory_switch = msg->data;
}
void ref_position_cb(const geometry_msgs::Vector3::ConstPtr& msg)
{
    ref_position << msg->x, msg->y, msg->z;
}
void ref_linvel_LRwheel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    ref_linvel_LRwheel.clear();
    ref_linvel_LRwheel.insert(ref_linvel_LRwheel.end(), msg->data.begin(), msg->data.end());
}
void ref_heading_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_heading = msg->data;
}
void ref_headingrate_cb(const std_msgs::Float64::ConstPtr& msg)
{
    ref_headingrate = msg->data;
}
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double roll = 0, pitch = 0, yaw = 0;
    current_att_quat = {
        msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w};
    current_att_mat.setRotation(current_att_quat);
    current_att_mat.getRPY(roll, pitch, yaw);
    //ROS_INFO("aaa");
    current_pos_att = {msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, roll, pitch, yaw};
}
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    current_vel_rate = {msg->twist.linear.x,
                        msg->twist.linear.y,
                        msg->twist.linear.z,
                        msg->twist.angular.x,
                        msg->twist.angular.y,
                        msg->twist.angular.z};
}
void wheelLinVel_cb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    current_wheelLinVel.clear();
    current_wheelLinVel.insert(current_wheelLinVel.end(), msg->data.begin(), msg->data.end());
}

void freeAreaCallBack(const static_collision_avoidance::collision_free_polygon& msg){
    //ROS_INFO("LMPCC::FreeAreaCallBack");
    collision_free_a1x = msg.collision_free_a1x;
    collision_free_a1y = msg.collision_free_a1y;
    collision_free_a2x = msg.collision_free_a2x;
    collision_free_a2y = msg.collision_free_a2y;
    collision_free_a3x = msg.collision_free_a3x;
    collision_free_a3y = msg.collision_free_a3y;
    collision_free_a4x = msg.collision_free_a4x;
    collision_free_a4y = msg.collision_free_a4y;

    collision_free_C1 = msg.collision_free_C1;
    collision_free_C2 = msg.collision_free_C2;
    collision_free_C3 = msg.collision_free_C3;
    collision_free_C4 = msg.collision_free_C4;

}

void publish_cmd(double commandstruct[2])
{
    //    commandstruct.control_vec = {-0.5,-0.5};
    std_msgs::Float64 wheel_vel_msg;
    wheel_vel_msg.data = commandstruct[0];
    nmpc_cmd_wheelLinVel_1_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_3_pub.publish(wheel_vel_msg);

    wheel_vel_msg.data = commandstruct[1];
    nmpc_cmd_wheelLinVel_2_pub.publish(wheel_vel_msg);
    nmpc_cmd_wheelLinVel_4_pub.publish(wheel_vel_msg);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mpc_jackal");
    ros::NodeHandle nh;

    //ros::param::get("/mocap", mocap_topic_part);
    pred_traj_pub_ = nh.advertise<nav_msgs::Path>("predicted_trajectory",1);

    //ref_trajectory_switch_sub = nh.subscribe<std_msgs::Bool>("/ref_trajectory/switch", 1, ref_trajectory_switch_cb);
    //ref_position_sub = nh.subscribe<geometry_msgs::Vector3>("/ref_trajectory/pose", 1, ref_position_cb);
    //ref_linvel_LRwheel_sub =
    //    nh.subscribe<std_msgs::Float64MultiArray>("/ref_trajectory/linearvel_wheel", 1, ref_linvel_LRwheel_cb);
    //ref_heading_sub = nh.subscribe<std_msgs::Float64>("/ref_trajectory/heading", 1, ref_heading_cb);
    //ref_headingrate_sub = nh.subscribe<std_msgs::Float64>("/ref_trajectory/heading_rate", 1, ref_headingrate_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mocap/pose", 1, pos_cb);
    vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mocap/velocity_body", 1, vel_cb);
    wheelLinVel_sub =
        nh.subscribe<std_msgs::Float64MultiArray>("/mocap/wheel_velocity_body", 1, wheelLinVel_cb);
    collision_free_sub = nh.subscribe("collision_constraints", 1, freeAreaCallBack);
    // ----------
    // Publishers
    // ----------
    nmpc_cmd_wheelLinVel_1_pub = nh.advertise<std_msgs::Float64>("/joint1_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_2_pub = nh.advertise<std_msgs::Float64>("/joint2_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_3_pub = nh.advertise<std_msgs::Float64>("/joint3_velocity_controller/command", 1, true);
    nmpc_cmd_wheelLinVel_4_pub = nh.advertise<std_msgs::Float64>("/joint4_velocity_controller/command", 1, true);
  
    ros::Rate loop_rate(1 / 0.01);

    for (int i = 0; i < (int)(1 / 0.01); ++i)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    while (ros::ok())
    {

    double currentState[6];
    double collision_param[12*rti_num];
    double tracking_goal[7];

    currentState[0] = current_pos_att.at(0);
        ROS_INFO("x %.3f", current_pos_att.at(0));
    currentState[1] = current_pos_att.at(1);
        ROS_INFO("y %.3f", current_pos_att.at(1));
    currentState[2] = current_pos_att.at(5);
    currentState[3] = current_vel_rate.at(5);
    currentState[4] = (current_wheelLinVel.at(0) + current_wheelLinVel.at(2)) / 2.0;
    currentState[5] = (current_wheelLinVel.at(1) + current_wheelLinVel.at(3)) / 2.0;
    currentState[6] = 0.0000001;
            // Setting up references [x,y,cos(psi),sin(psi),omega,vx_l,vx_r]
    tracking_goal[0] = 5;
    tracking_goal[1] = 1.2;
    tracking_goal[2] = cos(0);
    tracking_goal[3] = sin(0);
    tracking_goal[4] = 0;
    tracking_goal[5] = 0;
    tracking_goal[6] = 0;
    
        collision_free_a1x.resize(rti_num);
        collision_free_a1y.resize(rti_num);
        collision_free_a2x.resize(rti_num);
        collision_free_a2y.resize(rti_num);
        collision_free_a3x.resize(rti_num);
        collision_free_a3y.resize(rti_num);
        collision_free_a4x.resize(rti_num);
        collision_free_a4y.resize(rti_num);
        collision_free_C1.resize(rti_num);
        collision_free_C2.resize(rti_num);
        collision_free_C3.resize(rti_num);
        collision_free_C4.resize(rti_num);
        
    for (int i=0; i<rti_num; i++) {
    collision_param[i*12+0] = collision_free_a1x[i];
    collision_param[i*12+1] = collision_free_a1y[i];
    collision_param[i*12+2] = collision_free_a2x[i];
    collision_param[i*12+3] = collision_free_a2y[i];
    collision_param[i*12+4] = collision_free_a3x[i];
    collision_param[i*12+5] = collision_free_a3y[i];
    collision_param[i*12+6] = collision_free_a4x[i];
    collision_param[i*12+7] = collision_free_a4y[i];
    collision_param[i*12+8] = collision_free_C1[i];
    collision_param[i*12+9] = collision_free_C2[i];
    collision_param[i*12+10] = collision_free_C3[i];
    collision_param[i*12+11] = collision_free_C4[i];
    }
    double* outputs;
    double solutions[2];
   //double* pred_traj;
    // ROS_INFO("collision %.3f %.3f %.3f %.3f", collision_param[348], collision_param[349], collision_param[350], collision_param[351]);
    outputs = myMpcSolver.solve_mpc(currentState, collision_param, tracking_goal);
    solutions[0]=outputs[0];
    solutions[1]=outputs[1];
    publish_cmd(solutions);
    ROS_INFO("%.3f",outputs[0]);
    nav_msgs::Path pred_traj_;
    pred_traj_.poses.resize(rti_num);

    for (int k = 0; k < rti_num; k++)
    {
        pred_traj_.poses[k].pose.position.x = outputs[k * 3 + 0+2]; //x
   //         ROS_INFO("out %.3f",outputs[k * 3 + 0+2]);

        pred_traj_.poses[k].pose.position.y = outputs[k * 3 + 1+2]; //y
		pred_traj_.poses[k].pose.orientation.z = outputs[k * 3 + 2+2]; //theta
    }
   // ROS_INFO("pred_traj %.3f", pred_traj[0]);

    pred_traj_pub_.publish(pred_traj_);
    ROS_INFO("vel %.3f", solutions[0]);
    ROS_INFO("vel %.3f", solutions[1]);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
