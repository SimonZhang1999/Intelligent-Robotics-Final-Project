#include <mpc_control/mpc_controllor.hpp>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <memory>
#include <math.h>
#include <vector>
#include <thread>

std::shared_ptr<MPCNavigation> mpc_control_ptr;

void Start(){
    while (true){
        // Check for traces
        if(mpc_control_ptr->HasTrajectory()){
            // Check for new tracks
            if(mpc_control_ptr->JudgeTajectory()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            while(true){
                if(!mpc_control_ptr->MPC_init()){
                    if(mpc_control_ptr->JudgeTajectory()) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    continue;
                }
                int traj_count = mpc_control_ptr->getTrajectorySize();
                int traj_now_count = 0;
                while(true){
                    if(mpc_control_ptr->TargetPointDection()){
                        break;
                    }
                    ros::Time time_last = ros::Time::now();
                    mpc_control_ptr->MPC_start(traj_now_count);
                    traj_now_count++;
                    if(traj_now_count > traj_count - 10)
                        break;
                    ros::Time time_now = ros::Time::now();
                }
                if(mpc_control_ptr->TargetPointDection()){
                    mpc_control_ptr->CmdZero();
                    break;
                }
            }
            mpc_control_ptr->CmdZero();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "mpc_control_node");
    ros::NodeHandle nh("~");

    mpc_control_ptr = std::make_shared<MPCNavigation>(nh);

    // Open thread, parallel operation for closed loop feedback
    std::thread beginthread(Start);
    beginthread.detach();

    ros::spin(); 

    return 0;
}