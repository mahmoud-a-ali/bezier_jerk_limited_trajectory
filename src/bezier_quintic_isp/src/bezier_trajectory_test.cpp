#include <ros/ros.h>
#include "bezier_quintic_segment.h"
#include "bezier_trajectory.h"

//#include<normal_toppra_traj_instant_1.h>
#include<normal_toppra_traj_instant_3.h>
//#include<test_trajectory_max_jrk.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
const double max_pos=180, max_vel= 130, max_acc=250, max_jrk=1000, frq=125; // pos, vel, acc, jrk max limits


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_trajectory_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //============ read trajectory ==========
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();
    // extract pos, vel ,acc
    std::vector< std::vector<double> > P_jt_wpt, V_jt_wpt, A_jt_wpt, T_jt_wpt, durations;
    P_jt_wpt.resize(n_jts);
    V_jt_wpt.resize(n_jts);
    A_jt_wpt.resize(n_jts);
    T_jt_wpt.resize(n_jts);
    durations.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            P_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );
            V_jt_wpt[jt].push_back( traj.points[pt].velocities[jt] );
            A_jt_wpt[jt].push_back( traj.points[pt].accelerations[jt] );
            T_jt_wpt[jt].push_back(0);
        }
    }
    double t_start =T_jt_wpt[0][0];
    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)
            durations[jt].push_back( T_jt_wpt[jt][seg+1] - T_jt_wpt[jt][seg]);






    //create trajectory ------------------------------------------------------------
    for(int jt=0; jt<n_jts ; jt++){ //n_jts-1
        ROS_INFO_STREAM("================");
        ROS_INFO_STREAM("jt: "<< jt );
        ROS_INFO_STREAM("================");
//        ROS_INFO_STREAM("###original times:  ");
//        for (int i=0; i<T_jt_wpt[jt].size(); i++)
//            ROS_INFO_STREAM("  "<< T_jt_wpt[jt][i]);


        // initiate bezier_trajectory
        bezier_trajectory traj(P_jt_wpt[jt]); // just waypoint without vel, acc
//        bezier_trajectory traj(P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt], durations[jt], t_start );
        T_jt_wpt[jt] =traj.update_waypoints_times(max_pos, max_vel, max_acc, max_jrk);
        traj.print_attributes();
        // using sample function to sample trajectory
        std::vector<double> T_vec, POS, VEL,ACC, JRK;
        traj.trajectory_states( frq, T_vec, POS, VEL,ACC, JRK );

        //plot trajectory after update times
        plt::figure(1);
        plt::subplot(2, 2, 1);
        plt::named_plot( "pos",T_vec, POS); plt::named_plot( "waypoints",T_jt_wpt[jt], P_jt_wpt[jt], "r*");
        plt::title("pos"); plt::grid(true); // plt::legend();
        plt::subplot(2, 2, 2);
        plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
        plt::subplot(2, 2, 3);
        plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
        plt::subplot(2, 2, 4);
        plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");

    }

     plt::show();

}
















