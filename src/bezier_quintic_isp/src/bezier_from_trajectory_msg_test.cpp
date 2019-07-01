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
    ros::init(argc, argv, "bezier_from_trajectory_msg_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //read trajectory from header file
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();
    double t=traj.points[0].positions[0];

    bezier_robot_trajectory rbt_traj(traj);

    std::vector<double> sync_times, sync_durations;
    sync_times = rbt_traj.update_waypoints_times(max_pos, max_vel, max_acc, max_jrk);
    rbt_traj.compute_durations_from_times( sync_times, sync_durations);


    // use sample function to sample all joint trajectories and plot them
    std::vector<double> sampled_time, pos, vel, acc, jrk ;
    std::vector<std::vector<double>>  POS, VEL,ACC, JRK;
    POS.resize(n_jts);  VEL.resize(n_jts);
    ACC.resize(n_jts);  JRK.resize(n_jts);
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

    //extract the original waypoint to check they are synchronized or not
    std::vector< std::vector<double> > Pos_jt_wpt;
    Pos_jt_wpt.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++)
        for(int pt=0; pt<n_pts; pt++)
            Pos_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );


    for (int jt=0; jt<n_jts; jt++) {
        plt::figure(1);
        plt::subplot(2, 2, 1);
        plt::named_plot( "pos",sampled_time, POS[jt]); plt::named_plot( "waypoints", sync_times, Pos_jt_wpt[jt], "r*");
        plt::title("pos"); plt::grid(true); plt::title("pos"); //plt::legend();
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












