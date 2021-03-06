//
// Created by yuwei on 1/31/20.
//
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>
#include <pm_mpc/spline.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <nav_msgs/Odometry.h>

#include <pm_mpc/track.h>
#include <Eigen/Sparse>
#include "OsqpEigen/OsqpEigen.h"
#include <pm_mpc/occupancy_grid.h>

using namespace std;
using namespace Eigen;

const int nx = 3;
const int nu = 2;
const double CAR_LENGTH = 0.35;


enum rviz_id{
    CENTERLINE,
    CENTERLINE_POINTS,
    THETA_EST,
    PREDICTION,
    BORDERLINES,
    TRAJECTORY_REF,
    TRAJECTORIES,
    MAX_THETA,
    DEBUG
};


typedef struct Stage{
    Matrix<double, nx, nx> Q;
    Matrix<double, nu, nu> R;
    Matrix<double, nx, 1> fx;
    Matrix<double, nu, 1> fu;
    Matrix<double, nx, nx> Ad;
    Matrix<double, nx, nu> Bd;
    Matrix<double, nx, 1> dv_cost_dx;
}Stage;


class PMMPC{

public:
    PMMPC(ros::NodeHandle& nh);

private:
    ros::NodeHandle nh_;
    ros::Publisher track_viz_pub_;
    ros::Publisher trajectories_viz_pub_;
    ros::Publisher PMMPC_viz_pub_;
    ros::Publisher drive_pub_;
    ros::Publisher debugger_pub_;

    ros::Subscriber odom_sub_;
    ros::Subscriber rrt_sub_;
    ros::Subscriber map_sub_;
    /*Paramaters*/
    string pose_topic;
    string drive_topic;
    double Ts;
    double dtheta;
    int speed_num;
    int steer_num;
    int N;
    double SPEED_MAX;
    double STEER_MAX;
    double ACCELERATION_MAX;
    double DECELERATION_MAX;
    double MAP_MARGIN;
    double SPEED_THRESHOLD;
    // MPC params
    double q_x;
    double q_y;
    double q_yaw;
    double r_v;
    double r_steer;
    double q_s;
    Matrix<double, nx, nx> Q;
    Matrix<double, nu, nu> R;


    Track track_;
    tf::Transform tf_;
    tf::Vector3 car_pos_;
    double yaw_;
    double car_theta_;
    double speed_m_;
    vector<Vector3d> trajectory_ref_;
    Vector2d input_ref_;
    double last_speed_cmd_;

    vector<geometry_msgs::Point> rrt_path_;
    nav_msgs::OccupancyGrid map_;
    nav_msgs::OccupancyGrid map_updated_;

    vector<geometry_msgs::Point> border_lines;

    void getParameters(ros::NodeHandle& nh);
    void init_occupancy_grid();
    void visualize_centerline();
    void compute_trajectory_table();
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);
    void select_trajectory();
    void simulate_dynamics(Vector3d& state, Vector2d& input, double dt, Vector3d& new_state);

    void execute_MPC();

    void get_linearized_dynamics(Matrix<double,nx,nx>& Ad, Matrix<double,nx, nu>& Bd,
            Matrix<double,nx,1>& x_op, Matrix<double,nu,1>& u_op, double t);

    Vector2d global_to_track(double x, double y, double yaw, double theta);
    Vector3d track_to_global(double e_y, double e_yaw, double theta);

    void applyControl(VectorXd& QPSolution);
    void visualize_trajectories(int low, int high);
    void visualize_mpc_solution(VectorXd& QPSolution);

    void rrt_path_callback(const visualization_msgs::Marker::ConstPtr &path_msg);
    void map_callback(const nav_msgs::OccupancyGrid::Ptr &map_msg);


};