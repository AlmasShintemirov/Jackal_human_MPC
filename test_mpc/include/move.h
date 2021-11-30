#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <std_msgs/Float64.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "solver_holder.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>
#include "static_collision_avoidance/static_environment.h"
#include <nav_msgs/Path.h>

// Subscribers
ros::Subscriber ref_trajectory_switch_sub;
ros::Subscriber ref_position_sub;
ros::Subscriber ref_velocity_sub;
ros::Subscriber ref_linvel_LRwheel_sub;
ros::Subscriber ref_heading_sub;
ros::Subscriber ref_headingrate_sub;
ros::Subscriber pos_sub;
ros::Subscriber vel_sub;
ros::Subscriber wheelLinVel_sub;
ros::Subscriber collision_free_sub;

// Publishers
ros::Publisher nmpc_cmd_wheelLinVel_1_pub;
ros::Publisher nmpc_cmd_wheelLinVel_2_pub;
ros::Publisher nmpc_cmd_wheelLinVel_3_pub;
ros::Publisher nmpc_cmd_wheelLinVel_4_pub;

ros::Publisher pred_traj_pub_;

std::string mocap_topic_part="/mocap";
bool ref_trajectory_switch;
Eigen::Vector3d ref_position, ref_velocity;
std::vector<double> ref_linvel_LRwheel;
double ref_heading, ref_headingrate;
std::vector<double> ref_trajectory;
double t, t_pc_loop;

tf::Quaternion current_att_quat;
tf::Matrix3x3 current_att_mat;
std::vector<double> current_pos_att;
std::vector<double> current_vel_rate;
std::vector<double> current_wheelLinVel;
std::vector<double> current_states;

std::vector<double> collision_free_C1, collision_free_C2, collision_free_C3, collision_free_C4, collision_free_a1x ,collision_free_a1y, collision_free_a2x ,collision_free_a2y, collision_free_a3x ,collision_free_a3y, collision_free_a4x ,collision_free_a4y , collision_free_xmin, collision_free_xmax, collision_free_ymin, collision_free_ymax;
