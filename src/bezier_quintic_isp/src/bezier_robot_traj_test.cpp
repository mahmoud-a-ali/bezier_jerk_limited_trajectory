#include <ros/ros.h>
#include "bezier_quintic_segment.h"
#include "bezier_trajectory.h"

//#include<normal_toppra_traj_instant_1.h>
#include<normal_toppra_traj_instant_3.h>
//#include<test_trajectory_max_jrk.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

const double max_pos=180, max_vel= 130, max_acc=250, max_jrk=1000, frq=125; // pos, vel, acc, jrk max limits


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_robot_traj_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //read trajectory from header file
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();


    // extract Positions, Velocities ,Accelerations, Times from that trajectory and store them in std::vectors
    std::vector< std::vector<double> > Pos_jt_wpt, Vel_jt_wpt, ACC_jt_wpt, Times_jt_wpt, durations;
    Pos_jt_wpt.resize(n_jts);
    Vel_jt_wpt.resize(n_jts);
    ACC_jt_wpt.resize(n_jts);
    Times_jt_wpt.resize(n_jts);
    durations.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            Pos_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );
            Vel_jt_wpt[jt].push_back( traj.points[pt].velocities[jt] );
            ACC_jt_wpt[jt].push_back( traj.points[pt].accelerations[jt] );
//            Times_jt_wpt[jt].push_back(0);
            Times_jt_wpt[jt].push_back( traj.points[pt].time_from_start.toSec()/1e9); //already converted in stored traj
        }
    }
    double t_start =Times_jt_wpt[0][0];
    std::vector<double> required_duration,  sync_durations, sync_times;
    required_duration.resize(n_pts-1);
    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)
            durations[jt].push_back( Times_jt_wpt[jt][seg+1] - Times_jt_wpt[jt][seg]); // vector of durations for each joint
    required_duration = durations[0]; //one duration vector for all joints



    // initiate bezier robot trajectory
    bezier_robot_trajectory rbt_traj(Pos_jt_wpt); // if you have only a path without time/velor acc
//    bezier_robot_trajectory rbt_traj(Pos_jt_wpt, Vel_jt_wpt, ACC_jt_wpt, durations[0], t_start );
    sync_times = rbt_traj.update_waypoints_times(max_pos, max_vel, max_acc, max_jrk);
    //update timefor each trajectory and then compute synchronized times
    rbt_traj.compute_durations_from_times( sync_times, sync_durations);


    // use sample function to sample all joint trajectories and plot them
    std::vector<double> sampled_time, pos, vel, acc, jrk ;
    std::vector<std::vector<double>>  POS, VEL,ACC, JRK;
    POS.resize(n_jts);  VEL.resize(n_jts);
    ACC.resize(n_jts);  JRK.resize(n_jts);
    double t=t_start;
    while(t< sync_times.back() ){
        // use  sample method to sample trajectory
        rbt_traj.sample_bezier_robot_trajectory( t, pos, vel, acc, jrk );
        for (int jt=0; jt<n_jts; jt++) {
            POS[jt].push_back( pos[jt]);
            VEL[jt].push_back(vel[jt]);
            ACC[jt].push_back( acc[jt]);
            JRK[jt].push_back( jrk[jt]);
        }
        sampled_time.push_back(t);
        t+= 1/frq;

    }

    // plot all joit trajectories
    for (int jt=0; jt<n_jts; jt++) {
        plt::figure(1);
        plt::subplot(2, 2, 1);
        plt::named_plot( "pos",sampled_time, POS[jt]); plt::named_plot( "waypoints",sync_times, Pos_jt_wpt[jt], "r*");
        plt::title("pos"); plt::grid(true); // plt::legend();
        plt::subplot(2, 2, 2);
        plt::named_plot( "vel",sampled_time, VEL[jt]); plt::grid(true); plt::title("vel");
        plt::subplot(2, 2, 3);
        plt::named_plot( "acc",sampled_time, ACC[jt]); plt::grid(true); plt::title("acc");
        plt::subplot(2, 2, 4);
        plt::named_plot( "jrk",sampled_time, JRK[jt]); plt::grid(true); plt::title("jrk");
    }

    rbt_traj.print_attributes();
    plt::show();


} //end of main






//    for(int seg=0; seg<n_pts-1; seg++)
//        ROS_INFO_STREAM("required_duration["<<seg<<"] = "<< required_duration[seg]);

//    for(int seg=0; seg<n_pts-1; seg++)
//        ROS_INFO_STREAM("sync_durations["<<seg<<"] = "<< sync_durations[seg]);
//    ROS_INFO_STREAM("total_time = "<< sync_times.back()- t_start);












