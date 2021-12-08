#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <math.h>
#include <vector>
#include <iostream>

#include <mini_snap/PolynomialTrajectory.h>
#include <mini_snap/mini_snap.hpp>
#include <mini_snap/poly_nomial_trajectory.hpp>

double mean_vel = 0;
mini_snap::PolynomialTrajectory poly_pub_topic;
ros::Publisher poly_coef_pub;
ros::Publisher goal_list_pub;
ros::Publisher mini_snap_trajectory_pub;
bool get_new_path = false;
int id = 0;
std::shared_ptr<MiniSnapCloseform> mini_snap_solver_ptr;
std::shared_ptr<PolynomialTraj>    poly_traj_ptr;

void path_callback(const nav_msgs::Path::ConstPtr& msg);
void solve_min_snap(const nav_msgs::Path::ConstPtr& msg);
void deal_poly(const mini_snap::PolynomialTrajectory& msg);
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double& diff_time);


double last_yaw_, last_yaw_dot_;
bool receive_traj_ = false;
int traj_id_;
double traj_duration_, time_forward_;

void path_callback(const nav_msgs::Path::ConstPtr& msg){
    solve_min_snap(msg);
}

void solve_min_snap(const nav_msgs::Path::ConstPtr& msg){
    
    std::vector<Eigen::Vector3d> waypoints;
    for(int i_count = 0; i_count < msg->poses.size(); i_count++ ){
        Eigen::Vector3d wp;
        wp << msg->poses[i_count].pose.position.x,
              msg->poses[i_count].pose.position.y,
                                          0;
        waypoints.push_back(wp);
    }
    //start_yaw, final_yaw;
    double start_yaw_, final_yaw_,roll,pitch; 
    tf::Quaternion start_q;
    tf::Quaternion final_q;
    tf::quaternionMsgToTF(msg->poses.front().pose.orientation, start_q);
    tf::quaternionMsgToTF(msg->poses.back().pose.orientation, final_q);
    tf::Matrix3x3(start_q).getRPY(roll,pitch, start_yaw_);
    tf::Matrix3x3(final_q).getRPY(roll,pitch, final_yaw_);

    mini_snap_solver_ptr->Init(waypoints, mean_vel);
    mini_snap_solver_ptr->calMinsnap_polycoef();
    Eigen::MatrixXd poly_coef = mini_snap_solver_ptr->getPolyCoef();
    Eigen::MatrixXd dec_vel = mini_snap_solver_ptr->getDecVel();
    Eigen::VectorXd time = mini_snap_solver_ptr->getTime();

    poly_pub_topic.num_segment = msg->poses.size() - 1;
    poly_pub_topic.coef_x.clear();
    poly_pub_topic.coef_y.clear();
    poly_pub_topic.coef_z.clear();
    poly_pub_topic.time.clear();
    poly_pub_topic.trajectory_id = msg->poses.size();


    //By time
    for (int i = 0; i < time.size(); i++){ 
        for (int j = (i + 1) * 8 - 1; j >= i * 8; j--){
            poly_pub_topic.coef_x.push_back(poly_coef(j, 0));
            poly_pub_topic.coef_y.push_back(poly_coef(j, 1));
            poly_pub_topic.coef_z.push_back(poly_coef(j, 2));
        }
        poly_pub_topic.time.push_back(time(i));
    }

    poly_pub_topic.start_yaw = start_yaw_;
    poly_pub_topic.final_yaw = final_yaw_;
    poly_pub_topic.header.frame_id = "world";
    poly_pub_topic.header.stamp = ros::Time::now();
    deal_poly(poly_pub_topic);
    poly_coef_pub.publish(poly_pub_topic);
}

void deal_poly(const mini_snap::PolynomialTrajectory& msg){
    
    poly_traj_ptr->reset();
    int idx;
    for (int i = 0; i < msg.num_segment; i++){
        std::vector<double> cx, cy, cz;
        for (int j = 0; j < msg.num_order + 1; j++){
            idx = i * (msg.num_order + 1) + j;
            cx.push_back(msg.coef_x[idx]);
            cy.push_back(msg.coef_y[idx]);
            cz.push_back(msg.coef_z[idx]);
        }
        poly_traj_ptr->addSegment(cx, cy, cz, msg.time[i]);
    }
    poly_traj_ptr->init();
    traj_id_ = msg.trajectory_id;
    double traj_duration_ = poly_traj_ptr->getTimeSum();

    Eigen::Vector3d pos(Eigen::Vector3d::Zero());
    Eigen::Vector3d vel(Eigen::Vector3d::Zero());
    Eigen::Vector3d acc(Eigen::Vector3d::Zero());
    Eigen::Vector3d jerk(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0, 0);

    // Time allocation, MPC operation frequency 0.1s
    double diff_time = 0.1;
    
    geometry_msgs::PoseArray traj_pts;
    traj_pts.header.frame_id = "map";
    traj_pts.header.stamp = ros::Time::now();

    nav_msgs::Path global_trajecory_msg;
    global_trajecory_msg.header.frame_id = "map";
    global_trajecory_msg.header.stamp = ros::Time::now();

    Eigen::Vector3d last_pose(Eigen::Vector3d::Zero());
    for(double t_cur = 0.0; t_cur < traj_duration_-0.1; t_cur+=0.1){
        if (t_cur < traj_duration_ && t_cur >= 0.0){
            pos = poly_traj_ptr->evaluate(t_cur);
            vel = poly_traj_ptr->evaluateVel(t_cur);
            acc = poly_traj_ptr->evaluateAcc(t_cur);
            jerk = poly_traj_ptr->evaluateJerk(t_cur);
            yaw_yawdot = calculate_yaw(t_cur, pos, diff_time);
        }
        else if (t_cur >= traj_duration_){
            pos = poly_traj_ptr->evaluate(traj_duration_);
            yaw_yawdot.first = last_yaw_;
            yaw_yawdot.second = 0;
        }
        else{
            ROS_WARN("[my Traj server]: invalid time.");
        }
        geometry_msgs::PoseStamped path_point;
        geometry_msgs::Pose        opt_pt;
        // Solve parametric equations of x,y, and z with respect to t
        opt_pt.position.x = path_point.pose.position.x = pos[0];
        opt_pt.position.y = path_point.pose.position.y = pos[1];
        opt_pt.position.z = path_point.pose.position.z = pos[2];
        if(t_cur == 0.0){
            auto pose_q = tf::createQuaternionMsgFromYaw(msg.start_yaw);
            opt_pt.orientation = path_point.pose.orientation = pose_q;
            last_pose = pos;
        }
        else if(t_cur == traj_duration_-0.1){
            auto pose_q = tf::createQuaternionMsgFromYaw(msg.final_yaw);
            opt_pt.orientation = path_point.pose.orientation = pose_q;
        }
        else{
            Eigen::Vector3d pose_vector = pos - last_pose;
            double cur_yaw = std::atan2(double(pose_vector(1)),double(pose_vector(0)));
            auto pose_q = tf::createQuaternionMsgFromYaw(cur_yaw);
            opt_pt.orientation = path_point.pose.orientation = pose_q;
            last_pose = pos;
        }
        
        traj_pts.poses.push_back(opt_pt);
        global_trajecory_msg.poses.push_back(path_point);
    }
    mini_snap_trajectory_pub.publish(global_trajecory_msg);
    goal_list_pub.publish(traj_pts);
}


std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double& diff_time){
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? poly_traj_ptr->evaluate(t_cur + time_forward_) - pos : poly_traj_ptr->evaluate(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * diff_time;
    if (yaw_temp - last_yaw_ > PI){
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change){
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else{
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / diff_time;
        }
    }
    else if (yaw_temp - last_yaw_ < -PI){
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change){
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else{
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / diff_time;
        }
    }
    else{
        if (yaw_temp - last_yaw_ < -max_yaw_change){
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change){
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else{
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / diff_time;
        }
    }
    // nieve LPF
    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; 
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "mini_snap_node");
    ros::NodeHandle nh("~");

    nh.param<double>("mean_vel", mean_vel, 1.0);
    
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("/rrt_star_path", 1, &path_callback);
    // Find the x and y trajectories with respect to t
    goal_list_pub            = nh.advertise<geometry_msgs::PoseArray>("/goal_list", 10);
    poly_coef_pub            = nh.advertise<mini_snap::PolynomialTrajectory>("/poly_coefs", 10);/
    mini_snap_trajectory_pub = nh.advertise<nav_msgs::Path>("/global_trajectory", 10);

    mini_snap_solver_ptr = std::make_shared<MiniSnapCloseform>();
    poly_traj_ptr        = std::make_shared<PolynomialTraj>();

    poly_pub_topic.num_order = 7;
    poly_pub_topic.start_yaw = 0;
    poly_pub_topic.final_yaw = 0;
    poly_pub_topic.mag_coeff = 0;
    poly_pub_topic.order.push_back(0);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}