#include <mpc_control/mpc_control.hpp>

MPCNavigation::MPCNavigation(ros::NodeHandle &nh){

	nh.param<std::string>("base_frmae", base_frame, "/base_link");
	nh.param<std::string>("world_frame", world_frame, "/map");
	nh.param<std::string>("local_map_topic", local_map_topic, "/map");
	
	nh.param<double>("theta_weight", theta_weight, 0.01);
	nh.param<double>("max_vel",max_vel,0.4);
	nh.param<double>("min_vel",min_vel,-0.3);
	nh.param<double>("max_ang",max_ang,0.7);
	nh.param<double>("min_ang",min_ang,-0.7);


	odom_timer      = nh.createTimer(ros::Duration(0.05), &MPCNavigation::OdomCallback, this);
	local_map_sub   = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap",10,&MPCNavigation::LocalMapCallback, this);
	path_sub        = nh.subscribe<nav_msgs::Path>("/global_trajectory", 1, &MPCNavigation::PathCallback, this);
	cmd_pub 	    = nh.advertise<geometry_msgs::Twist>("/cmd_vel",1);
	forward_path_pub= nh.advertise<nav_msgs::Path>("/forward_trajectory", 10, true);
	judge_path_pub  = nh.advertise<nav_msgs::Path>("/judge_trajectory", 10, true);
	replan          = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	map_info_ptr   = std::make_shared<MapInfo>();
	
}

// Gets the robot's position in the map
void MPCNavigation::OdomCallback(const ros::TimerEvent&){

	tf::StampedTransform transform;
    static bool first_step = false;
    static Eigen::Vector3d last_pose;
    static ros::Time time_last = ros::Time::now();
    try{
        listener.waitForTransform(world_frame.c_str(),base_frame.c_str(), ros::Time(), ros::Duration(5.0));
        listener.lookupTransform(world_frame.c_str(),base_frame.c_str(),ros::Time(0), transform);
    }
    catch(tf::TransformException &ex){
        return;
    }

    geometry_msgs::Quaternion q;
    tf::Quaternion quat;
    q.w = transform.getRotation().w();
    q.x = transform.getRotation().x();
    q.y = transform.getRotation().y();
    q.z = transform.getRotation().z();
	tf::quaternionMsgToTF(q, quat);
	double roll, pitch, robot_yaw;
	tf::Matrix3x3(quat).getRPY(roll,pitch,robot_yaw);
	Position << transform.getOrigin().x(), 
				transform.getOrigin().y(),
								robot_yaw;
	if(!first_step){
		Velocity << 0, 0;
		last_pose = Position;
		first_step = true;
	}
	// Computing speed
	else{ 
		ros::Time time_now = ros::Time::now();
		double diff_time = (time_now - time_last).toSec();
		double v_liner = std::sqrt((Position - last_pose).transpose() * (Position - last_pose)) / diff_time;
		double v_angle = DiffRawFromLastToNow(Position(2), last_pose(2));
		if(v_liner < 1e-4 ) v_liner = 0;
		if(v_angle < 1e-4 ) v_angle = 0;
		if(v_liner > 1.1 ) v_liner = 1;
		if(v_angle > 0.7 ) v_liner = 0.7;
		Velocity << v_liner, v_angle;
		last_pose = Position;
	}
}

// The resulting trajectory is called back
void MPCNavigation::PathCallback(const nav_msgs::Path::ConstPtr& msg){ nimi snap
	std::vector<Eigen::Vector3d> waypoints;
	path_list_get.clear();
    for(int i_count = 0; i_count < msg->poses.size(); i_count++ ){
        Eigen::Vector3d wp;
        double yaw,roll,pitch;
        tf::Quaternion path_point_q;
        tf::quaternionMsgToTF(msg->poses[i_count].pose.orientation, path_point_q);
        tf::Matrix3x3(path_point_q).getRPY(roll,pitch, yaw);
        wp << msg->poses[i_count].pose.position.x,
              msg->poses[i_count].pose.position.y,
                                          	  yaw;
        path_list_get.push_back(wp);
    }
    get_path = true;
}

bool MPCNavigation::HasTrajectory(){
	bool get_path_ = get_path;
	get_path = false;
	return get_path_;
}

bool MPCNavigation::JudgeTajectory(){
	return get_path;
}

double MPCNavigation::DiffRawFromLastToNow(const double& now_yaw, const double& last_yaw){
	const double MPC_PI = 3.1415927;
	double diff_yaw = 0.0;

	if(last_yaw >= 0 && last_yaw <= MPC_PI &&
	   now_yaw  >= 0 && now_yaw  <= MPC_PI ){
		diff_yaw = now_yaw - last_yaw;
	}
	else if(last_yaw >= -MPC_PI && last_yaw <= 0 &&
	   		now_yaw  >= -MPC_PI && now_yaw  <= 0){
		diff_yaw = last_yaw - now_yaw;
	}
	else if(last_yaw >= -MPC_PI && last_yaw <= 0 &&
	   		now_yaw  >= 0 && now_yaw  <= MPC_PI){
		if(now_yaw - last_yaw - MPC_PI >= 0)
			diff_yaw = last_yaw - now_yaw;
		else
			diff_yaw = now_yaw - last_yaw;
	}
	else if(last_yaw >= 0 && last_yaw <= MPC_PI &&
	   		now_yaw  >= -MPC_PI && now_yaw  <= MPC_PI){
		if(last_yaw - now_yaw - MPC_PI >= 0)
			diff_yaw = last_yaw - now_yaw;
		else
			diff_yaw = now_yaw - last_yaw;
	}
	return diff_yaw;
}

void MPCNavigation::LocalMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
	global_map = *msg;
}

int MPCNavigation::JudgeOccupied(const std::vector<Eigen::Vector3d>& local_path){
	for(int i = 0; i < local_path.size(); i++){
		if(!map_info_ptr->isObsFree(local_path[i](0), local_path[i](1))){
			return i;
		}
	}
	return -1;
}

int MPCNavigation::GetCurrentIndex(){

	Eigen::Vector2d current_pose = Position.block<2,1>(0,0);
	std::vector<double> all_distance;
	for(int index=0; index < path_list_get.size()-10; index++){
		Eigen::Vector2d path_pose = path_list_get[index].block<2,1>(0,0);
		double distance = (path_pose - current_pose).transpose() * (path_pose - current_pose);
		all_distance.push_back(distance);
	}

	double min_distance = 1e6;
	int return_index = -1;
	for(int index=0; index < all_distance.size(); index++){
		if(all_distance[index] < min_distance){
			min_distance = all_distance[index];
			return_index = index;
		}
	}
	if(min_distance > 0.5){
		geometry_msgs::PoseStamped re_goal_msg;
		re_goal_msg.pose.position.x = path_list_get.back()[0];
		re_goal_msg.pose.position.y = path_list_get.back()[1];
		auto pose_q = tf::createQuaternionMsgFromYaw(path_list_get.back()[2]);
    	re_goal_msg.pose.orientation = pose_q;
		replan.publish(re_goal_msg);
	}
	return return_index;

}

bool MPCNavigation::MPC_init(){

	BuildMap();
	TrajectorySelfBuild();
	path_list.clear();
	if(traj_sum_.size() == 0)
		return false;
	path_list = TrajectoryGet();

	if(int(path_list.size()) < 5){
		return false;
	}
	// tarjectory size :
	double path_long_size = path_list.size();
	X_real.resize(3,path_long_size);
	X_piao.resize(3,path_long_size);

	U_real.resize(2,path_long_size);
	U_piao.resize(2,path_long_size);
	
	X_real.block<3,1>(0,0) = Position;		
	X_piao.block<3,1>(0,0) = X_real.block<3,1>(0,0) - path_list[0];	// Real location - Reference location
	
	U_real.block<2,1>(0,0) = Velocity;
	U_piao.block<2,1>(0,0) = U_real.block<2,1>(0,0);	// True speed - Reference speed

	N = 6;

	Eigen::Matrix3d q_;
	q_ << 1,   0,   		0,
		  0,   1,   		0,
		  0,   0,theta_weight;	
	// The smaller the theta weight is, the faster the rapier is with the trajectory, but the global tracking angular velocity jitter is too large
	// If the weight of Theta is 1, it means that the equidistance is more dependent on the trajectory
	Q.resize(3*N,3*N);
	for(int i = 0; i < N; i++)
		for(int j=0; j < N; j++)
			Q.block<3,3>(i*3,j*3) = q_;

	R.resize(2*N,2*N);
	for(int i = 0; i < 2*N; i++)
		for(int j=0; j < 2*N; j++)
			if(i == j)
				R(i,j) = 1;
				else R(i,j) = 0;
	R = 0.1 * R;

	A_info.resize(N*2,N*2);
	for(int i = 0; i < N*2; i++)
		for(int j=0; j < N*2; j++)
			if(i == j) A_info(i,j) = 1;
			else A_info(i,j) = 0;

	
	return true;	
}

void MPCNavigation::MPC_start(int i_count){

	ros::Rate rate_(10);

	double theta = path_list[i_count](2);
	double v 	 = U_real(0,i_count);
	double w     = U_real(1,i_count);


	Eigen::Matrix3d A_piao = Eigen::Matrix3d::Zero();
	A_piao << 1,		0,	-v * delta_t * sin(theta),
		      0,		1,	 v * delta_t * cos(theta),
		      0,		0,	 				    	1;
	
	Eigen::Matrix<double, 3, 2> B_piao = Eigen::Matrix<double, 3, 2>::Zero();
	B_piao << delta_t * cos(theta),			0,
		 	  delta_t * sin(theta),			0,
			       			     0,    delta_t;

	Eigen::MatrixXd PHI;
	PHI.resize(N*3,3);

	for(int i=0; i < N; i++){
		Eigen::Matrix3d A_pow = Eigen::Matrix3d::Identity();
		for(int j=0; j <= i; j++){
			A_pow *= A_piao;
		}
		PHI.block<3,3>(i*3,0) = A_pow;
	}

	Eigen::MatrixXd THETA;
	THETA.resize(3*N,2*N);
	for(int i =0; i < THETA.rows(); i++)
		for(int j =0; j < THETA.cols(); j++)
			THETA(i,j) = 0;
	for(int k =0; k < N; k++){
		for(int i=0; i < N-k; i++){
			Eigen::Matrix3d A_pow = Eigen::Matrix3d::Identity();
			for(int j=0; j <= i; j++){
				A_pow *= A_piao;
			}
			THETA.block<18,2>(0,k*2).block<3,2>(i*3+3*k,0) = A_pow * B_piao;
		}	
	}	

	Eigen::MatrixXd H;
	H.resize(2*N,2*N);
	for(int i =0; i < H.rows(); i++)
		for(int j =0; j < H.cols(); j++)
			H(i,j) = 0;

	H = 2 * (THETA.transpose() * Q * THETA + R);
	H = (H + H.transpose()) /2;

	Eigen::VectorXd f(2*N);
	f = 2 * THETA.transpose() * Q * PHI * X_piao.block<3,1>(0,i_count);

	Eigen::SparseMatrix<double> Hessian;
	Hessian.resize(H.rows(), H.cols());
	Hessian = setosqpMatrix(H);

	Eigen::SparseMatrix<double> LinearMatrix;
	LinearMatrix.resize(A_info.rows(), A_info.cols());
	LinearMatrix = setosqpMatrix(A_info);
	
	Eigen::VectorXd u_min(N*2);
	Eigen::Vector2d u_min_;
	u_min_(0) = min_vel;
	u_min_(1) = min_ang;
	for(int i = 0; i < N; i++){
		u_min.block<2,1>(2*i,0) = u_min_;
	}
	Eigen::VectorXd u_max(N*2);
	Eigen::Vector2d u_max_;
	u_max_(0) = max_vel;
	u_max_(1) = max_ang;
	for(int i = 0; i < N; i++){
		u_max.block<2,1>(2*i,0) = u_max_;
	}

	Eigen::VectorXd u_real(2*N);
	for(int i = 0; i < N; i++)
		u_real.block<2,1>(2*i,0) = U_real.block<2,1>(0,i_count);
	
	Eigen::VectorXd lb(N*2);
	lb = u_min - u_real;
	Eigen::VectorXd ub(N*2);
	ub = u_max - u_real;

	OsqpEigen::Solver solver;
	solver.settings()->setWarmStart(true);

	solver.data()->setNumberOfVariables(2*N);
	solver.data()->setNumberOfConstraints(2*N);
	Eigen::VectorXd  U_Optimization(2*N);
	U_Optimization = solver.getSolution();
    
    U_piao.block<2,1>(0,i_count + 1) = U_Optimization.block<2,1>(0,0);

    Eigen::Vector2d OUTPUT_state_;

    OUTPUT_state(0) = v + U_piao.block<2,1>(0,i_count+1)(0);
    OUTPUT_state(1) = w + U_piao.block<2,1>(0,i_count+1)(1);

    U_real.block<2,1>(0,i_count + 1) = OUTPUT_state;
    
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = OUTPUT_state(0);
    cmd_vel.angular.z = OUTPUT_state(1);
    // Continuous transmission speed, lasts 0.02 seconds
    cmd_pub.publish(cmd_vel);
    rate_.sleep();
    
    // Read the current position state of the odometer
    X_real.block<3,1>(0,i_count+1) = Position;

    if(Position[2] - path_list[i_count+1][2] <= 3.1415 && Position[2] - path_list[i_count+1][2] >= 0.0){
    	double z_delta = Position[2] - path_list[i_count+1][2];
    	Eigen::Vector3d temp_position(Position[0], Position[1], z_delta);
    	X_piao.block<3,1>(0,i_count+1) = Eigen::Vector3d((X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[0],
    												     (X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[1],
    												     z_delta);
    }
    else if(Position[2] - path_list[i_count+1][2] > 3.1415 && Position[2] - path_list[i_count+1][2] <= 6.29){
    	double z_delta = -(6.28 - (Position[2] - path_list[i_count+1][2]));
    	Eigen::Vector3d temp_position(Position[0], Position[1], z_delta);
    	X_piao.block<3,1>(0,i_count+1) = Eigen::Vector3d((X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[0],
    												     (X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[1],
    												     z_delta);
    }
    else if(Position[2] - path_list[i_count+1][2] >= -3.1415 && Position[2] - path_list[i_count+1][2] < 0.0){
    	double z_delta = Position[2] - path_list[i_count+1][2];
    	Eigen::Vector3d temp_position(Position[0], Position[1], z_delta);
    	X_piao.block<3,1>(0,i_count+1) = Eigen::Vector3d((X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[0],
    												     (X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[1],
    												     z_delta);
    }
    else if(Position[2] - path_list[i_count+1][2] < -3.1415 && Position[2] - path_list[i_count+1][2] >= -6.29){
    	double z_delta = (6.28 + (Position[2] - path_list[i_count+1][2]));
    	Eigen::Vector3d temp_position(Position[0], Position[1], z_delta);
    	X_piao.block<3,1>(0,i_count+1) = Eigen::Vector3d((X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[0],
    												     (X_real.block<3,1>(0,i_count+1) - path_list[i_count+1])[1],
    												     z_delta);
    }

    return;
}

int MPCNavigation::getTrajectorySize(){
	return path_list.size() - 10;
}

Eigen::SparseMatrix<double> MPCNavigation::setosqpMatrix(const Eigen::MatrixXd &matrix_){
	
	int row = matrix_.rows();
	int col = matrix_.cols();

	Eigen::SparseMatrix<double> matrix_return;
	matrix_return.resize(row,col);
	for(int i = 0; i < row; i++)
		for(int j = 0; j < col; j++)
			matrix_return.insert(i,j) = matrix_(i,j);
	return matrix_return; 
}

bool MPCNavigation::TargetPointDection(){
	Eigen::Vector2d target_coord(path_list.back()[0], path_list.back()[1]);
	Eigen::Vector2d target_coord_2(path_list_get.back()[0], path_list_get.back()[1]);
	Eigen::Vector2d local_coord = Position.block<2,1>(0,0);
	if((target_coord - local_coord).transpose() * (target_coord - local_coord) < 0.1 * 0.1){
		return true;
	}
	else if((target_coord_2 - local_coord).transpose() * (target_coord_2 - local_coord) < 0.1 * 0.1){
		return true;
	}
	else
		return false;
}

void MPCNavigation::TrajectorySelfBuild(){
	// Generate 21 paths based on the current location
	Eigen::Matrix3d A_ = Eigen::Matrix3d::Identity();
	Eigen::Matrix<double, 3, 2> B_ = Eigen::Matrix<double, 3, 2>::Zero();
	std::map<int,std::vector<Eigen::Vector3d>> traj_sum;
	
	Eigen::Vector3d now_position = Position; 


	double delta_yaw = 1.6 / 21;
	for(int single_idx = -10; single_idx <= 10; single_idx ++){
		for(int forward_idx = -1; forward_idx <= 1 ; forward_idx++){
			if(forward_idx == 0) continue;
			Eigen::Vector3d tra_position = now_position;
			std::vector<Eigen::Vector3d> traj_single;
			traj_single.push_back(tra_position);
			for(int node = 0; node < 10; node++){
				double theta_ = tra_position(2);
				B_ << cos(theta_), 0,
					  sin(theta_), 0,
					  			0, 1;
				Eigen::Vector2d u_;
				u_ << double(forward_idx) * max_vel, single_idx * delta_yaw;
				tra_position = A_ * tra_position + 0.1 * B_ * u_;
				traj_single.push_back(tra_position);
			}
			// If there are obstacles on the track, delete the track for local judgment and do not add it
			if(JudgeOccupied(traj_single) != -1) continue;
			traj_sum.insert( std::make_pair(int(single_idx+10), traj_single) );
		}
	}
	traj_sum_.clear();
	traj_sum_ = traj_sum;
}

std::vector<Eigen::Vector3d> MPCNavigation::TrajectoryGet(){
	int traj_num = traj_sum_.size();
	int max_path_list_num = path_list_get.size();
	std::map<int,std::vector<Eigen::Vector3d>>::iterator it;
	int current_idx = GetCurrentIndex();
	std::vector<Eigen::Vector3d> current_path_list;
	for(int num = 0; num < 15; num++){
		if(current_idx > max_path_list_num-15) break;
		auto now_node = path_list_get[current_idx];
		current_path_list.push_back(now_node);
		current_idx++;
	}
	if( (current_path_list.front().block<2,1>(0,0) - current_path_list.back().block<2,1>(0,0)).transpose() *
		(current_path_list.front().block<2,1>(0,0) - current_path_list.back().block<2,1>(0,0)) < 0.09){
		for(int num = 0; num < 8; num++){
			if(current_idx > max_path_list_num-15) break;
			auto now_node = path_list_get[current_idx];
			current_path_list.push_back(now_node);
			current_idx++;
		}
	}


	std::map<double, int> score_vec;
	for(it = traj_sum_.begin(); it != traj_sum_.end(); it++ ){
		auto judge_tra = it->second;
		double score2 = 0.0;
		for(int i = 0; i < judge_tra.size(); i++){
			for(int j = 0; j < current_path_list.size(); j++){
				score2 +=  double((judge_tra[i].block<2,1>(0,0) - current_path_list[j].block<2,1>(0,0)).transpose() * 
						   (judge_tra[i].block<2,1>(0,0) - current_path_list[j].block<2,1>(0,0)));
			}
		}
		score_vec.insert( std::make_pair(score2, it->first));
	}

	std::vector<Eigen::Vector3d> getTrajectory = traj_sum_[score_vec.begin()->second];

	ForwardPathVis(getTrajectory);
	JudgePathVis(current_path_list);
	auto last_point = getTrajectory.back();
	for( int i = 0; i < 15; i++)
		getTrajectory.push_back(last_point);
	return getTrajectory;
}

void MPCNavigation::CmdZero(){
	geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    cmd_pub.publish(cmd_vel);
}
