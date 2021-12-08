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

#include <rrt_nav/graph_searcher.h>

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include "rrt_nav/backward.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

std::shared_ptr<RRTstarPreparatory> _RRTstar_preparatory;
ros::Publisher _grid_map_vis_pub;
ros::Publisher _RRTstar_path_vis_pub;
ros::Publisher _RRTstar_path_pub;

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace backward {
backward::SignalHandling sh;
}

struct map_info{
    int map_width;
    int map_height;
    double origin_x;
    double origin_y;
    double map_resolution;
    std::vector<std_msgs::Int8> map_data;
};

struct robot_coord{
    double position_x;
    double position_y;
    Eigen::Quaterniond q;
};

nav_msgs::OccupancyGrid global_map;
tf::TransformListener* listener;
robot_coord robot_odom;
std::string base_frame, map_frame;
bool get_new_map = false;
std::vector<Eigen::Vector3d> global_path;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void odom_callback(const ros::TimerEvent&);
map_info get_map();
robot_coord get_odom();
void visRRTstarPath(std::vector<Eigen::Vector3d> nodes );
bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt);
void dealpathtrajectory(std::vector<Eigen::Vector3d>& path);
void send_path(const Eigen::Quaterniond& begin_q, const Eigen::Quaterniond& end_q);



void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    global_map = *msg;
    get_new_map = true;
}

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){

    auto map_infomation = get_map();

    Eigen::Vector2d _map_lower, _map_upper;
    _map_lower << - std::abs(map_infomation.origin_x), - std::abs(map_infomation.origin_y);
    _map_upper << + map_infomation.map_width * map_infomation.map_resolution  - std::abs(map_infomation.origin_x), 
                  + map_infomation.map_height * map_infomation.map_resolution - std::abs(map_infomation.origin_y);

    std::cout << _map_lower.transpose() << ", " << _map_upper.transpose() << std::endl;
    double _inv_resolution = 1.0 / map_infomation.map_resolution;
    
    int _max_x_id = (int)(map_infomation.map_width * _inv_resolution);
    int _max_y_id = (int)(map_infomation.map_width * _inv_resolution);
    double _origin_x = map_infomation.origin_x;
    double _origin_y = map_infomation.origin_y;

    _RRTstar_preparatory = std::make_shared<RRTstarPreparatory>();
    _RRTstar_preparatory -> initGridMap(map_infomation.map_resolution, _map_lower, _map_upper, _max_x_id, _max_y_id);

    sensor_msgs::PointCloud2 map_vis;
    pcl::PointCloud<pcl::PointXYZ> cloud_vis;
    pcl::PointXYZ pt;
    for(int row = 0; row < map_infomation.map_height; row++){
        for(int col = 0; col < map_infomation.map_width; col++){
            int p = int(map_infomation.map_data[col+row*map_infomation.map_width].data);
            
            if (p > 50) {
                raw_map_matrix(row,col) = -1;        //occupied
            }
            else if(p <=50){
                raw_map_matrix(row,col) = 1;        //free
                continue;
            }
            Eigen::Vector2d cor_round = _RRTstar_preparatory->coordRounding(Eigen::Vector2i(col, row));
            _RRTstar_preparatory->setObs(cor_round(0), cor_round(1));
            pt.x = cor_round(0);
            pt.y = cor_round(1);
            pt.z = 0;
            cloud_vis.points.push_back(pt);
        }
    }
    cloud_vis.width    = cloud_vis.points.size();
    cloud_vis.height   = 1;
    cloud_vis.is_dense = true;

    pcl::toROSMsg(cloud_vis, map_vis);

    map_vis.header.frame_id = "map";
    _grid_map_vis_pub.publish(map_vis);
    get_new_map = false;

    Eigen::Vector3d _start_pt;
    Eigen::Vector3d _target_pt;

    auto start_pt = get_odom();

    start_pt = get_odom();
    _start_pt << start_pt.position_x , start_pt.position_y, 0;
    _target_pt << msg->pose.position.x , msg->pose.position.y, 0;
    ROS_WARN("start pose : (%3f, %3f), valid : %d",
                _start_pt(0), _start_pt(1),_RRTstar_preparatory->isObsFree(_start_pt(0), _start_pt(1))); 
    ROS_WARN("target pose: (%3f, %3f), valid : %d",
                _target_pt(0), _target_pt(1), _RRTstar_preparatory->isObsFree(_target_pt(0), _start_pt(1)));

    global_path.clear();

    if (pathFinding(_start_pt, _target_pt)) {
        Eigen::Quaterniond end_q;
        end_q.x() = msg->pose.orientation.x;
        end_q.y() = msg->pose.orientation.y;
        end_q.z() = msg->pose.orientation.z;
        end_q.w() = msg->pose.orientation.w;
        send_path(start_pt.q, end_q);
    }
    else
        return flase;
}



void odom_callback(const ros::TimerEvent&){

    tf::StampedTransform transform;
    static bool first_step = false;
    try{
        auto rostime = ros::Time(0);
        if(!first_step){
            ros::Duration(1).sleep();
            first_step = true;
        }
        else
            ros::Duration(0.1).sleep();
        listener->lookupTransform(map_frame.c_str(),base_frame.c_str(),rostime,transform);
    }
    catch(tf::TransformException &ex){
        return;
    }
    Eigen::Quaterniond q;
    q.w() = transform.getRotation().w();
    q.x() = transform.getRotation().x();
    q.y() = transform.getRotation().y();
    q.z() = transform.getRotation().z();

    robot_coord current_T = {transform.getOrigin().x(), transform.getOrigin().y(), q};
    robot_odom = current_T;

}

map_info get_map(){

    int map_width  = global_map.info.width;
    int map_height = global_map.info.height;
    double origin_x   = global_map.info.origin.position.x;
    double origin_y   = global_map.info.origin.position.y;
    double map_resolution = global_map.info.resolution;

    std::vector<std_msgs::Int8> map_data_;
    for(int i = 0; i < global_map.data.size(); i++){
        std_msgs::Int8 value;
        value.data = global_map.data[i];
        map_data_.push_back(value);
    }
    map_info mapInfo = {map_width, map_height, origin_x, origin_y, map_resolution, map_data_};
    return mapInfo;

}

robot_coord get_odom(){

    return robot_odom;

}


bool pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt){
    
    auto map_infomation = get_map();
    //coordinate transform
    double _x_size = map_infomation.map_width * map_infomation.map_resolution;          
    double _y_size = map_infomation.map_height * map_infomation.map_resolution;
    Eigen::Vector2d _map_lower, _map_upper;
    _map_lower << - std::abs(map_infomation.origin_x), - std::abs(map_infomation.origin_y);
    _map_upper << + map_infomation.map_width * map_infomation.map_resolution  - std::abs(map_infomation.origin_x), 
                  + map_infomation.map_height * map_infomation.map_resolution - std::abs(map_infomation.origin_y);
    double resolution = map_infomation.map_resolution;

    ob::StateSpacePtr space(new ob::RealVectorStateSpace(2)); 
    ob::RealVectorBounds bounds(2);
    //Set the boundary
    bounds.setLow(0, _map_lower(0) );
    bounds.setLow(1, _map_lower(1) );

    bounds.setHigh(0, + _map_upper(0) );
    bounds.setHigh(1, + _map_upper(1) );

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
    si->setup();
    
    //Start point coordinates
    ob::ScopedState<> start(space);
    start[0] = (&start_pt)->operator[](0)  ;
    start[1] = (&start_pt)->operator[](1)  ;

    //End point coordinates
    ob::ScopedState<> goal(space);
    goal[0] = (&target_pt)->operator[](0)  ;
    goal[1] = (&target_pt)->operator[](1)  ;

    auto pdef(std::make_shared<ob::ProblemDefinition>(si));
    pdef->setStartAndGoalStates(start, goal);
    pdef->setOptimizationObjective(getPathLengthObjective(si));
    ob::PlannerPtr optimizingPlanner(new og::RRTstar(si));
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();
    ob::PlannerStatus solved = optimizingPlanner->solve(1.0);
    if (solved){
        og::PathGeometric* path = pdef->getSolutionPath()->as<og::PathGeometric>();
        std::vector<Eigen::Vector3d> path_points;
        for (size_t path_idx = 0; path_idx < path->getStateCount(); path_idx++){
            const ob::RealVectorStateSpace::StateType *state = path->getState(path_idx)->as<ob::RealVectorStateSpace::StateType>();
            auto x = (*state)[0];
            auto y = (*state)[1];
            Eigen::Vector3d temp_mat(x,y,0);
            path_points.push_back(temp_mat);
        }
        std::vector<Eigen::Vector3d> path_points_clear;
        dealpathtrajectory(path_points);
        visRRTstarPath(path_points_clear);
        visRRTstarPath(path_points);
        if(path_points.size() == 2){
            auto x_1 = path_points[0];
            auto x_2 = path_points[1];
            path_points.pop_back();
            path_points.push_back((x_1+x_2)/2.0);
            path_points.push_back(x_2);
        }
        global_path = path_points;
        return true;    
    }
    else
        return false;
}

void dealpathtrajectory(std::vector<Eigen::Vector3d>& path){
    std::vector<Eigen::Vector3d> new_path;
    for(int i = 0; i < path.size() - 1; i ++){
        double distance_ab = std::sqrt((path[i+1] - path[i]).transpose() * (path[i+1] - path[i]));
        if(distance_ab > 0.5){
            int count_ab = int(distance_ab / 1.0);
            Eigen::Vector3d vector_ab = (path[i+1] - path[i]) / double(count_ab);
            new_path.push_back(path[i]);
            for(int j=1; j < count_ab+1 ; j++){
                new_path.push_back(path[i]+vector_ab*j);
            }
            if(std::sqrt((path[i+1] - new_path.back()).transpose() * (path[i+1] - new_path.back())) < 1.5)
                new_path.pop_back();
        }
        else
            new_path.push_back(path[i]);
    }
    new_path.push_back(path[path.size()-1]);
    path.clear();
    path = new_path;
}


void send_path(const Eigen::Quaterniond& begin_q, const Eigen::Quaterniond& end_q){
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < global_path.size(); i++){
        geometry_msgs::PoseStamped path_point;
        path_point.pose.position.x = global_path[i][0];
        path_point.pose.position.y = global_path[i][1];
        path_point.pose.position.z = 0;
        if(i == 0){
            path_point.pose.orientation.x = begin_q.x();
            path_point.pose.orientation.y = begin_q.y();
            path_point.pose.orientation.z = begin_q.z();
            path_point.pose.orientation.w = begin_q.w();
        }
        else if(i == global_path.size()-1){
            path_point.pose.orientation.x = end_q.x();
            path_point.pose.orientation.y = end_q.y();
            path_point.pose.orientation.z = end_q.z();
            path_point.pose.orientation.w = end_q.w();
        }
        else{
            path_point.pose.orientation.x = 0;
            path_point.pose.orientation.y = 0;
            path_point.pose.orientation.z = 0;
            path_point.pose.orientation.w = 1;
        }
        path_msg.poses.push_back(path_point);
    }

    _RRTstar_path_pub.publish(path_msg);

}

void visRRTstarPath(std::vector<Eigen::Vector3d> nodes ){
    
    auto map_infomation = get_map();
    double _x_size = map_infomation.map_width * map_infomation.map_resolution;
    double _y_size = map_infomation.map_height * map_infomation.map_resolution;

    visualization_msgs::Marker Points, Line; 
    Points.header.frame_id = Line.header.frame_id = "map";
    Points.header.stamp    = Line.header.stamp    = ros::Time::now();
    Points.ns              = Line.ns              = "demo_node/RRTstarPath";
    Points.action          = Line.action          = visualization_msgs::Marker::ADD;
    Points.pose.orientation.w = Line.pose.orientation.w = 1.0;
    Points.id = 0;
    Line.id   = 1;
    Points.type = visualization_msgs::Marker::POINTS;
    Line.type   = visualization_msgs::Marker::LINE_STRIP;

    Points.scale.x = map_infomation.map_resolution/2; 
    Points.scale.y = map_infomation.map_resolution/2;
    Line.scale.x   = map_infomation.map_resolution/2;

    //points are green and Line Strip is blue
    Points.color.g = 1.0;
    Points.color.a = 1.0;
    Line.color.b   = 0.0;
    Line.color.r   = 1.0;
    Line.color.a   = 1.0;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Eigen::Vector3d coord = nodes[i];
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        Points.points.push_back(pt);
        Line.points.push_back(pt);
    }
    _RRTstar_path_vis_pub.publish(Points);
    _RRTstar_path_vis_pub.publish(Line); 
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "rrt_node");
    ros::NodeHandle nh("~");

    nh.param<std::string>("base_frame", base_frame, "/base_link");
    nh.param<std::string>("map_frame", map_frame, "/map");

    tf::TransformListener listener_;
    listener = &listener_;

    ros::Subscriber map_sub  = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/global_costmap/costmap", 1000, &map_callback);
    //Subscribe to the global cost map
    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000, &goal_callback);

    _grid_map_vis_pub        = nh.advertise<sensor_msgs::PointCloud2>("grid_map_vis", 1, true);
    _RRTstar_path_vis_pub    = nh.advertise<visualization_msgs::Marker>("RRTstar_path_vis",1, true);
    _RRTstar_path_pub        = nh.advertise<nav_msgs::Path>("/rrt_star_path",1000);

    ros::Timer      odom_timer = nh.createTimer(ros::Duration(0.2), &odom_callback);
    //Keep repeating the cycle until you get its location

    ros::Rate r(10);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
    }


    return 0;
}