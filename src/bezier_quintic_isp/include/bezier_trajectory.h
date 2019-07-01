#ifndef BEZIER_TRAJECTORY_H
#define BEZIER_TRAJECTORY_H
#include "bezier_quintic_segment.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

#include<vector>
#include<math.h>
#include<iostream>
class bezier_trajectory
{
private:
    double max_pos_=0, max_vel_=0, max_acc_=0, max_jrk_=0, t_start_=0;
    std::vector<double> positions_, velocities_, accelerations_, jerks_,  durations_,times_;
    int n_points_=0;
public:
    bezier_trajectory();
    bezier_trajectory(std::vector<double> waypoints_pos, std::vector<double> waypoints_vel,
            std::vector<double> waypoints_acc, std::vector<double> waypoints_durations, double start_time){
       init( waypoints_pos,  waypoints_vel, waypoints_acc,  waypoints_durations, start_time);
    }

    bezier_trajectory(std::vector<double> waypoints_pos, std::vector<double> waypoints_vel,std::vector<double> waypoints_acc){
        init( waypoints_pos,  waypoints_vel, waypoints_acc,  durations_, t_start_);
    }

    bezier_trajectory(std::vector<double> waypoints_pos,  std::vector<double> waypoints_durations, double start_time){
        init( waypoints_pos,  velocities_, accelerations_,  waypoints_durations, start_time);
    }

    bezier_trajectory(std::vector<double> waypoints_pos){
        init( waypoints_pos,  velocities_, accelerations_,  durations_, t_start_);
    }



void init(std::vector<double> waypoints_pos, std::vector<double> waypoints_vel,
           std::vector<double> waypoints_acc, std::vector<double> waypoints_durations, double start_time_);

void set_inflection_pts_vel_to_zero();


void set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk);
//void set_duration_(std::vector<double> durations);

std::vector<double> update_waypoints_times();
std::vector<double> update_waypoints_times(double max_pos, double max_vel, double max_acc, double max_jrk);


int find_segment (double t);
void sample_bezier_trajectory(double t, std::vector<double> &state);
void trajectory_states(const double frq, std::vector<double> &T_vec, std::vector<double> &POS, std::vector<double> &VEL, std::vector<double> &ACC, std::vector<double> &JRK );
void print_attributes();


};







//===============================================
//bezier_robot_trajectory
//===============================================
class bezier_robot_trajectory
{
private:
    double max_pos_=0, max_vel_=0, max_acc_=0, max_jrk_=0, t_start_=0;
    std::vector<std::vector<double>> positions_, velocities_, accelerations_, jerks_,  durations_, times_;
    std::vector<double> sync_times_, sync_durations_;
    int n_joints_=0, n_points_=0;
    std::vector<bezier_trajectory> joint_trajectories_;
public:
    bezier_robot_trajectory();
    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
            std::vector<std::vector<double>> waypoints_acc, std::vector<double> waypoints_durations, double start_time){
       init( waypoints_pos,  waypoints_vel, waypoints_acc,  waypoints_durations, start_time);
    }

    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
            std::vector<std::vector<double>> waypoints_acc, std::vector<std::vector<double>> waypoints_durations, double start_time){
       init( waypoints_pos,  waypoints_vel, waypoints_acc,  waypoints_durations, start_time);
    }


    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
            std::vector<std::vector<double>> waypoints_acc, double start_time){
       init( waypoints_pos,  waypoints_vel, waypoints_acc,  durations_, start_time);
    }

    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
            std::vector<std::vector<double>> waypoints_acc){
       init( waypoints_pos,  waypoints_vel, waypoints_acc,  durations_, t_start_);
    }


    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos, double start_time){
        init(waypoints_pos, velocities_, accelerations_, sync_durations_, start_time );
    }


    bezier_robot_trajectory(std::vector<std::vector<double>> waypoints_pos){
        init(waypoints_pos, velocities_, accelerations_, sync_durations_, t_start_ );
    }

    bezier_robot_trajectory(trajectory_msgs::JointTrajectory msg){
        init_from_trajectory_msg( msg );
    }


    void init(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
              std::vector<std::vector<double>> waypoints_acc, std::vector<double> waypoints_durations, double start_time);
    void init(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
              std::vector<std::vector<double>> waypoints_acc, std::vector<std::vector<double>> waypoints_durations, double start_time);

    void init_from_trajectory_msg( trajectory_msgs::JointTrajectory msg );

    void set_vel_inflection_pts_to_zero();
    void set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk);
    std::vector<double> update_waypoints_times();
    std::vector<double> update_waypoints_times(double max_pos, double max_vel, double max_acc, double max_jrk);

    void sample_bezier_robot_trajectory(double t, std::vector<double> &POS, std::vector<double> &VEL,
                                                                           std::vector<double> &ACC, std::vector<double> &JRK);
    void print_attributes();



    static void generate_sync_durations( std::vector< std::vector<double> > traj_durations, std::vector<double> &sync_durations);
    static void compute_durations_from_times( std::vector<double> traj_times, std::vector<double>  &traj_durations);
    static void compute_times_from_durations(double t_start, std::vector<std::vector<double>> traj_durations, std::vector< std::vector<double> > &traj_times );
    static void compute_durations_from_times(std::vector< std::vector<double> > traj_times, std::vector< std::vector<double> > &traj_durations);
    static void compute_times_from_durations( double t_start, std::vector<double>  traj_durations,  std::vector<double>  &traj_times );

};

#endif // BEZIER_TRAJECTORY_H
