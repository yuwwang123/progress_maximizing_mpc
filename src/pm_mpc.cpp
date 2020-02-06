//
// Created by yuwei on 1/31/20.
//

#include <ros/ros.h>
#include <Eigen/Dense>
#include <pm_mpc/pm_mpc.h>
#include <unsupported/Eigen/MatrixFunctions>

const string file_name = "/home/yuwei/rcws/logs/yuwei_wp.csv";
const double RRT_INTERVAL = 0.1;

PMMPC::PMMPC(ros::NodeHandle &nh): nh_(nh), track_(Track(file_name, 0.5)){

    getParameters(nh_);
    init_occupancy_grid();

    odom_sub_ = nh_.subscribe(pose_topic, 10, &PMMPC::odom_callback, this);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    rrt_sub_ = nh_.subscribe("path_found", 1, &PMMPC::rrt_path_callback, this);
    map_sub_ = nh_.subscribe("map_updated", 1, &PMMPC::map_callback, this);

    track_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("track_centerline", 10);

    PMMPC_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("PMMPC", 10);
    debugger_pub_ = nh_.advertise<visualization_msgs::Marker>("Debugger", 10);

}

void PMMPC::getParameters(ros::NodeHandle &nh) {
    nh.getParam("pose_topic", pose_topic);
    nh.getParam("drive_topic", drive_topic);
    nh.getParam("N",N);
    nh.getParam("Ts",Ts);
    nh.getParam("dtheta",dtheta);

    nh.getParam("ACCELERATION_MAX", ACCELERATION_MAX);
    nh.getParam("DECELERATION_MAX", DECELERATION_MAX);
    nh.getParam("SPEED_MAX", SPEED_MAX);
    nh.getParam("STEER_MAX", STEER_MAX);

    nh.getParam("q_s",q_s);

    nh.getParam("MAP_MARGIN",MAP_MARGIN);
}

void PMMPC::init_occupancy_grid(){
    boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
    map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));
    if (map_ptr == nullptr){ROS_INFO("No map received");}
    else{
        map_ = *map_ptr;
        map_updated_ = map_;
        ROS_INFO("Map received");
    }
    ROS_INFO("Initializing occupancy grid for map ...");
    occupancy_grid::inflate_map(map_, MAP_MARGIN);
}

void PMMPC::visualize_centerline(){
    // plot waypoints
    visualization_msgs::Marker dots;
    dots.header.frame_id = "map";
    dots.id = rviz_id::CENTERLINE_POINTS;
    dots.ns = "centerline_points";
    dots.type = visualization_msgs::Marker::POINTS;
    dots.scale.x = dots.scale.y = 0.08;
    dots.scale.z = 0.04;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.color.r = 1.0;
    dots.color.a = 1.0;
    //dots.lifetime = ros::Duration();

    for (int i = 0; i < track_.centerline.size(); i++) {
        geometry_msgs::Point p;
        p.x = track_.centerline.at(i).x;
        p.y = track_.centerline.at(i).y;
        dots.points.push_back(p);
    }

    visualization_msgs::Marker line_dots;
    line_dots.header.stamp = ros::Time::now();
    line_dots.header.frame_id = "map";
    line_dots.id = rviz_id::CENTERLINE;
    line_dots.ns = "centerline";
    line_dots.type = visualization_msgs::Marker::LINE_STRIP;
    line_dots.scale.x = line_dots.scale.y = 0.02;
    line_dots.scale.z = 0.02;
    line_dots.action = visualization_msgs::Marker::ADD;
    line_dots.pose.orientation.w = 1.0;
    line_dots.color.b = 1.0;
    line_dots.color.a = 1.0;
    line_dots.points = dots.points;

    visualization_msgs::MarkerArray markers;
    markers.markers.push_back(dots);
    markers.markers.push_back(line_dots);

    track_viz_pub_.publish(markers);
}

void PMMPC::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg){
    /* process pose info */
    visualize_centerline();

    speed_m_ = odom_msg->twist.twist.linear.x;
    //speed_m_ = 3.2;

    tf::Quaternion q_tf;
    tf::quaternionMsgToTF(odom_msg->pose.pose.orientation, q_tf);
    tf::Matrix3x3 rot(q_tf);
    double roll, pitch;
    rot.getRPY(roll, pitch, yaw_);
    car_pos_ = tf::Vector3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 0.0);

    tf_.setOrigin(car_pos_);
    tf_.setRotation(q_tf);

    /*select reference trajectory*/
    /*low level MPC to track the reference*/
    execute_MPC();
}

//update corridor width
void PMMPC::map_callback(const nav_msgs::OccupancyGrid::Ptr &map_msg){
    visualization_msgs::Marker dots;
    dots.header.frame_id = "map";
    dots.id = rviz_id::DEBUG;
    dots.ns = "debug_points";
    dots.type = visualization_msgs::Marker::POINTS;
    dots.scale.x = dots.scale.y = 0.1;
    dots.scale.z = 0.1;
    dots.action = visualization_msgs::Marker::ADD;
    dots.pose.orientation.w = 1.0;
    dots.color.b = 1.0;
    dots.color.g = 0.5;
    dots.color.a = 1.0;

    vector<geometry_msgs::Point> path_processed;

    if(rrt_path_.empty()) return;
    for (int i=0; i< rrt_path_.size()-1; i++){
        path_processed.push_back(rrt_path_[i]);
        double dist = sqrt(pow(rrt_path_[i+1].x-rrt_path_[i].x, 2)
                           +pow(rrt_path_[i+1].y-rrt_path_[i].y, 2));
        if (dist < RRT_INTERVAL) continue;
        int num = static_cast<int>(ceil(dist/RRT_INTERVAL));
        for(int j=1; j< num; j++){
            geometry_msgs::Point p;
            p.x = rrt_path_[i].x + j*((rrt_path_[i+1].x - rrt_path_[i].x)/num);
            p.y = rrt_path_[i].y + j*((rrt_path_[i+1].y - rrt_path_[i].y)/num);
            path_processed.push_back(p);
        }
    }

    for (int i=0; i<path_processed.size(); i++){
        double theta = track_.findTheta(path_processed[i].x, path_processed[i].y, 0, true);
        Vector2d p_path(path_processed[i].x,path_processed[i].y);
        Vector2d p_proj(track_.x_eval(theta), track_.y_eval(theta));
        Vector2d p1, p2;
        int t=0;
        // search one direction until hit obstacle
        while(true){
            double x = (p_path + t*map_msg->info.resolution*(p_path-p_proj).normalized())(0);
            double y = (p_path + t*map_msg->info.resolution*(p_path-p_proj).normalized())(1);
            if(occupancy_grid::is_xy_occupied(*map_msg, x, y)){
                p1(0) = x; p1(1) = y;
                geometry_msgs::Point point;
                point.x = x; point.y = y;
                dots.points.push_back(point);
                break;
            }
            t++;
        }
        t=0;
        //search the other direction until hit obstacle
        while(true){
            double x = (p_path - t*map_msg->info.resolution*(p_path-p_proj).normalized())(0);
            double y = (p_path - t*map_msg->info.resolution*(p_path-p_proj).normalized())(1);
            if(occupancy_grid::is_xy_occupied(*map_msg, x, y)){
                p2(0) = x; p2(1) = y;
                geometry_msgs::Point point;
                point.x = x; point.y = y;
                dots.points.push_back(point);
                break;
            }
            t++;
        }
        double dx_dtheta = track_.x_eval_d(theta);
        double dy_dtheta = track_.y_eval_d(theta);
        double right_width = Vector2d(dy_dtheta, -dx_dtheta).dot(p1-p_proj)>0 ? (p1-p_proj).norm() : -(p1-p_proj).norm();
        double left_width = Vector2d(-dy_dtheta, dx_dtheta).dot(p2-p_proj)>0 ? (p2-p_proj).norm() : -(p2-p_proj).norm();
//        right_width = -0.15;
//        left_width =  0.4;

        track_.setHalfWidth(theta, left_width, right_width);
    }
    debugger_pub_.publish(dots);
}

void PMMPC:: rrt_path_callback(const visualization_msgs::Marker::ConstPtr &path_msg){
    rrt_path_ = path_msg->points;
}

void PMMPC::simulate_dynamics(Vector3d& state, Eigen::Vector2d& input, double dt, Eigen::Vector3d& new_state){
    VectorXd dynamics(state.size());
    dynamics(0) = input(0)*cos(state(2));
    dynamics(1) = input(0)*sin(state(2));
    dynamics(2) = tan(input(1))*input(0)/CAR_LENGTH;
    new_state = state + dynamics * dt;
};


void PMMPC::execute_MPC(){

    SparseMatrix<double> HessianMatrix((N+1)*(nx+nu) + (N+1) + nx*(N+1), (N+1)*(nx+nu) + (N+1) + nx*(N+1));
    SparseMatrix<double> constraintMatrix((N+1)*nx+ 2*(N+1)*nx +(N+1)*nu + (N+1)+ (N+1)*nx, (N+1)*(nx+nu) + (N+1) + nx*(N+1));

    VectorXd gradient((N+1)*(nx+nu) + (N+1) + nx*(N+1));
    gradient.setZero();

    VectorXd lower((N+1)*nx+ 2*(N+1)*nx +(N+1)*nu + (N+1)+ (N+1)*nx);
    VectorXd upper((N+1)*nx+ 2*(N+1)*nx +(N+1)*nu + (N+1)+ (N+1)*nx);

    Matrix<double,nx,1> x_k_ref;
    Matrix<double,nx,1> x_kp1_ref;
    Matrix<double,nu,1> u_k_ref;
    Matrix<double,nx,nx> Ad;
    Matrix<double,nx,nu> Bd;
    Matrix<double,nx,1> x0;

    double theta0 = track_.findTheta(car_pos_.x(), car_pos_.y(), 0, true);
    car_theta_ = theta0;
    x0 = global_to_track(car_pos_.x(), car_pos_.y(), yaw_, theta0);

    cout<<"yaw: "<<endl;
    cout<<yaw_<<endl;
    cout<< "x0: " << endl;
    cout<<x0 <<endl;

    for (int i=0; i<N+1; i++){        //0 to N
        double theta = theta0 + i*dtheta;
        double Ks = track_.getCenterlineCurvature(theta);

        x_k_ref << 0.0, 0.0;
        u_k_ref << Ks;
        double vx_ref = 2.0;

        get_linearized_dynamics(Ad, Bd, x_k_ref, u_k_ref, theta);
        /* form Hessian entries*/
        // cost does not depend on x0, only 1 to N
        if (i>0) {
            HessianMatrix.insert(i*nx+1, i*nx+1) = 0.5*(1-x_k_ref(0)*Ks)/vx_ref;  // coefficient for e_yaw^2
        }
        for(int row=0; row<nx; row++){
            HessianMatrix.insert((N+1)*(nx+nu)+ (N+1) +i*nx+row, (N+1)*(nx+nu)+ (N+1) +i*nx+row) = q_s;
        }

        /* form gradient vector*/
        gradient(i*nx) = -Ks/vx_ref;       // coefficient for e_y
        gradient((N+1)*(nx+nu)+i) = -(1-x_k_ref(0)*Ks)/pow(vx_ref, 2);  // coeff for vx
        /* form constraint matrix */
        if (i<N){
            x_kp1_ref << 0.0, 0.0;
            // Ad
            for (int row=0; row<nx; row++){
                for(int col=0; col<nx; col++){
                    constraintMatrix.insert((i+1)*nx+row, i*nx+col) = Ad(row,col);
                }
            }
            // Bd
            for (int row=0; row<nx; row++){
                for(int col=0; col<nu; col++){
                    //constraintMatrixTripletList.push_back(T(i*nx+row, (N+1)*nx+ i*nu+col, s.Bd(row,col)));
                    constraintMatrix.insert((i+1)*nx+row, (N+1)*nx+ i*nu+col) = Bd(row,col);
                }
            }
            lower.segment<nx>((i+1)*nx) = -x_kp1_ref + Ad*x_k_ref + Bd*u_k_ref;
            upper.segment<nx>((i+1)*nx) = -x_kp1_ref + Ad*x_k_ref + Bd*u_k_ref;
        }

        // -I for each x_k+1
        for (int row=0; row<nx; row++) {
            constraintMatrix.insert(i*nx+row, i*nx+row) = -1.0;
        }
        /* track boundary constraints */
        for (int row=0; row<nx; row++) {
            //  -inf <= x_k - s_k <= x_max
            constraintMatrix.insert((N+1)*nx + i*nx+row, i*nx+row) = 1.0;
            constraintMatrix.insert((N+1)*nx + i*nx+row, (N+1)*(nx+nu) + (N+1) + i*nx + row) = -1.0;
        }
        upper.segment<nx>((N+1)*nx + i*nx) << 0.4, M_PI/4.0; //track_.getLeftHalfWidth(theta), M_PI/4.0;
        lower.segment<nx>((N+1)*nx + i*nx) << -OsqpEigen::INFTY, -OsqpEigen::INFTY;

        for (int row=0; row<nx; row++) {
            //  x_min <= x_k + s_k <= inf
            constraintMatrix.insert((N+1)*nx + (N+1)*nx +i*nx+row, i*nx+row) = 1.0;
            constraintMatrix.insert((N+1)*nx + (N+1)*nx +i*nx+row, (N+1)*(nx+nu) + (N+1) + i*nx + row) = 1.0;
        }
        upper.segment<nx>((N+1)*nx + (N+1)*nx + i*nx) << OsqpEigen::INFTY, OsqpEigen::INFTY; //track_.getLeftHalfWidth(theta), M_PI/4.0;
        lower.segment<nx>((N+1)*nx + (N+1)*nx + i*nx) << -0.4, -M_PI/4.0; //-track_.getRightHalfWidth(theta), -M_PI/4.0;

        // u_min < u < u_max
        for (int row=0; row<nu; row++){
            constraintMatrix.insert((N+1)*nx+ 2*(N+1)*nx +i*nu+row, (N+1)*nx+i*nu+row) = 1.0;
        }
        // input bounds: speed and steer
        lower.segment<nu>((N+1)*nx+ 2*(N+1)*nx +i*nu) << -tan(STEER_MAX)/CAR_LENGTH;
        upper.segment<nu>((N+1)*nx+ 2*(N+1)*nx +i*nu) << tan(STEER_MAX)/CAR_LENGTH;

        // vx_min < vx_k <= vx_max
        constraintMatrix.insert((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + i, (N+1)*(nx+nu) + i) = 1.0;
        lower((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + i) = 0;
        upper((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + i) = SPEED_MAX;
        // s_k >= 0
        for (int row=0; row<nx; row++){
            constraintMatrix.insert((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + (N+1) + i*nx + row, (N+1)*(nx+nu)+ (N+1) +i*nx+row) = 1.0;
            lower((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + (N+1) + i*nx + row) = 0;
            upper((N+1)*nx + 2*(N+1)*nx + (N+1)*nu + (N+1) + i*nx + row) = OsqpEigen::INFTY;
        }
    }

    lower.head(nx) = -x0;  //x0
    upper.head(nx) = -x0;

    SparseMatrix<double> H_t = HessianMatrix.transpose();
    SparseMatrix<double> sparse_I((N+1)*(nx+nu) + (N+1) + nx*(N+1), (N+1)*(nx+nu) + (N+1) + nx*(N+1));
    sparse_I.setIdentity();
    HessianMatrix = 0.5*(HessianMatrix + H_t) + 0.000001*sparse_I;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables((N+1)*(nx+nu) + (N+1) + nx*(N+1));
    solver.data()->setNumberOfConstraints((N+1)*nx+ 2*(N+1)*nx +(N+1)*nu + (N+1)+ (N+1)*nx);

    if (!solver.data()->setHessianMatrix(HessianMatrix)) throw "fail set Hessian";
    if (!solver.data()->setGradient(gradient)){throw "fail to set gradient";}
    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix)) throw"fail to set constraint matrix";
    if (!solver.data()->setLowerBound(lower)){throw "fail to set lower bound";}
    if (!solver.data()->setUpperBound(upper)){throw "fail to set upper bound";}

    if(!solver.initSolver()){ cout<< "fail to initialize solver"<<endl;}

    if(!solver.solve()) {
        return;
    }
    VectorXd QPSolution = solver.getSolution();

    cout<<"Solution: "<<endl;
    cout<<QPSolution<<endl;

    applyControl(QPSolution);
    visualize_mpc_solution(QPSolution);

    solver.clearSolver();
}

Vector2d PMMPC::global_to_track(double x, double y, double yaw, double theta){
    double x_proj = track_.x_eval(theta);
    double y_proj = track_.y_eval(theta);
    double e_y = sqrt((x-x_proj)*(x-x_proj) + (y-y_proj)*(y-y_proj));
    double dx_dtheta = track_.x_eval_d(theta);
    double dy_dtheta = track_.y_eval_d(theta);
    e_y = dx_dtheta*(y-y_proj) - dy_dtheta*(x-x_proj) >0 ? e_y : -e_y;
    double e_yaw = yaw - atan2(dy_dtheta, dx_dtheta);
    if(e_yaw > M_PI) e_yaw -= 2*M_PI;
    if(e_yaw < -M_PI) e_yaw += 2*M_PI;

    return Vector2d(e_y, e_yaw);
}

Vector3d PMMPC::track_to_global(double e_y, double e_yaw, double theta){
    double dx_dtheta = track_.x_eval_d(theta);
    double dy_dtheta = track_.y_eval_d(theta);
    Vector2d proj(track_.x_eval(theta), track_.y_eval(theta));
    Vector2d pos = proj + Vector2d(-dy_dtheta, dx_dtheta).normalized()*e_y;
    double yaw = e_yaw + atan2(dy_dtheta, dx_dtheta);
    return Vector3d(pos(0), pos(1), yaw);
}


void PMMPC::get_linearized_dynamics(Matrix<double,nx,nx>& Ad, Matrix<double,nx, nu>& Bd, Matrix<double,nx,1>& x_op, Matrix<double,nu,1>& u_op, double t){

    Matrix<double,nx,nx> A;
    Matrix<double,nx,nu> B;
    double rho_s = track_.getCenterlineRadius(t);
    double Ks = track_.getCenterlineCurvature(t);

    A <<   0.0,                 (rho_s-x_op(0))/rho_s,      0.0
           -u_op(0)/rho_s,                        0.0,      0.0
           - Ks*u_op(1),                          0.0,      -(1-x_op(0)*Ks)*u_op(1)/(x_op(2)*x_op(2));

    B <<    0.0, 0.0,
            (rho_s-x_op(0))/rho_s, 0.0,
            0.0, (1-x_op(0)*Ks)/x_op(2);

    //Discretize with Euler approximation
    //Ad = Matrix2d::Identity() + A*dtheta
    //Bd = dtheta*B;

    Ad = (A*dtheta).exp();
    Bd = A.inverse()*(Ad - Matrix2d::Identity())*B;
}



void PMMPC::visualize_mpc_solution(VectorXd& QPSolution){
    geometry_msgs::Point p;

    visualization_msgs::Marker pred_dots;
    pred_dots.header.frame_id = "map";
    pred_dots.id = rviz_id::PREDICTION;
    pred_dots.ns = "predicted_positions";
    pred_dots.type = visualization_msgs::Marker::POINTS;
    pred_dots.scale.x = pred_dots.scale.y = pred_dots.scale.z = 0.08;
    pred_dots.action = visualization_msgs::Marker::ADD;
    pred_dots.pose.orientation.w = 1.0;
    pred_dots.color.g = 1.0;
    pred_dots.color.r = 1.0;
    pred_dots.color.a = 1.0;
    for (int i=0; i<N+1; i++){
        geometry_msgs::Point p;
        Vector3d res = track_to_global(QPSolution(i*nx), QPSolution(i*nx+1), car_theta_+i*dtheta);
        p.x = res(0);
        p.y = res(1);
        pred_dots.points.push_back(p);
    }

    visualization_msgs::MarkerArray mpc_markers;
    mpc_markers.markers.push_back(pred_dots);

    PMMPC_viz_pub_.publish(mpc_markers);
}

void PMMPC::applyControl(VectorXd& QPSolution) {

    double u0 = QPSolution((N+1)*nx);
    float steer = atan(u0*CAR_LENGTH);
    float speed = QPSolution((N+1)*(nx+nu));
    cout<<"steer_cmd: "<<steer<<endl;
    cout<<"speed_cmd: "<<speed<<endl;

    steer = min(steer, 0.41f);
    steer = max(steer, -0.41f);

    ackermann_msgs::AckermannDriveStamped ack_msg;
    ack_msg.drive.speed = speed;
    ack_msg.drive.steering_angle = steer;
    ack_msg.drive.steering_angle_velocity = 1.0;
    drive_pub_.publish(ack_msg);
    last_speed_cmd_ = speed;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "pm_mpc");
    ros::NodeHandle nh;
    PMMPC PMMPC(nh);
    ros::Rate rate(25);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}



