#include <ros/ros.h>
#include "bezier_quintic_segment.h"
#include<normal_toppra_traj_instant_1.h>
//#include<normal_toppra_traj_instant_3.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;



const double max_pos=180, max_vel= 130, max_acc=250, max_jrk=1000, frq=125; // pos, vel, acc, jrk max limits


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_segment_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //============ read trajectory ==========
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();
    // extract pos, vel ,acc
    std::vector< std::vector<double> > P_jt_wpt, V_jt_wpt, A_jt_wpt,T_jt_wpt;
    P_jt_wpt.resize(n_jts);
    V_jt_wpt.resize(n_jts);
    A_jt_wpt.resize(n_jts);
    T_jt_wpt.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            P_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );
            V_jt_wpt[jt].push_back( traj.points[pt].velocities[jt] );
            A_jt_wpt[jt].push_back( traj.points[pt].accelerations[jt] );
            T_jt_wpt[jt].push_back(0);
        }
    }


    //create segment
    int jt=4, seg_idx=0;
    double t_start=0,  old_duration=0.01;
    std::vector<double> start_state, end_state;
    start_state.resize(3);
    end_state.resize(3);

    for(int seg= seg_idx; seg<seg_idx+1 ; seg++){ //n_pts-1
        ROS_INFO_STREAM("=========================");
        ROS_INFO_STREAM("jt: "<< jt <<"    seg: "<< seg);
        ROS_INFO_STREAM("=========================");

        start_state[0]= P_jt_wpt[jt][seg] ;
        start_state[1]=V_jt_wpt[jt][seg] ;
        start_state[2]= A_jt_wpt[jt][seg] ;
        t_start = T_jt_wpt[jt][seg];

        end_state[0]= P_jt_wpt[jt][seg+1] ;
        end_state[1]= V_jt_wpt[jt][seg+1] ;
        end_state[2]= A_jt_wpt[jt][seg+1] ;


        bezier_quintic_segment traj_seg( t_start, old_duration, start_state, end_state );
        double new_duration = traj_seg.update_duration(max_pos, max_vel, max_acc, max_jrk);
        T_jt_wpt[jt][seg+1] = t_start + new_duration;
        traj_seg.print_attributes();
        std::vector<double> T_vec, POS, VEL,ACC, JRK;
        traj_seg.segment_states( frq, T_vec, POS, VEL,ACC, JRK );


// check extreme case when duration is greater than max and cause change in vel direction
//        bezier_quintic_segment test_seg( t_start, old_duration, start_state, end_state );
//        test_seg.set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
//        test_seg.compute_coef();
//        test_seg.compute_maxmin_times();
//        test_seg.print_attributes(); // without update provided duration
//        test_seg.check_monotonic();
//        ROS_INFO_STREAM("min_duration: "<< test_seg.compute_min_duration() );
//        test_seg.print_attributes(); //after setting it to min
//        ROS_INFO_STREAM("max_duration: "<< test_seg.compute_max_duration() );
//        ROS_INFO_STREAM("vel miita: "<< test_seg.check_vel_limit() );
//        ROS_INFO_STREAM("monotonic: "<< test_seg.check_monotonic() );
//        ROS_INFO_STREAM("limits, jrk, vel, : "<< test_seg.check_jerk_limit()<< test_seg.check_acc_limit()<< test_seg.check_vel_limit(); );
//        test_seg.print_attributes();
        //        traj_seg.segment_states( frq, T_vec, POS, VEL,ACC, JRK );




        // plot states
        plt::figure(1);
        plt::subplot(2, 2, 1);
        plt::named_plot( "pos",T_vec, POS); //plt::named_plot( "waypoints",T_jt_wpt[jt], P_jt_wpt[jt], "r*");

        plt::title("pos"); plt::grid(true); plt::title("pos"); //plt::legend();
        plt::subplot(2, 2, 2);
        plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
        plt::subplot(2, 2, 3);
        plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
        plt::subplot(2, 2, 4);
        plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");
        plt::show();

    }

//    plt::show();



}









//        plt::figure(1);
//        plt::named_plot( "pos",T_vec, POS); plt::grid(true); plt::title("pos");
//        plt::figure(2);
//        plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
//        plt::figure(3);
//        plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
//        plt::figure(4);
//        plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");


//start_state[0]= 1 ;
//start_state[1]= 0 ;
//start_state[2]= 0 ;
//t_start = 0;
//old_duration = 3.5; //3.55; //2.55

//end_state[0]= 2 ;
//end_state[1]= 01 ;
//end_state[2]= 0 ;















