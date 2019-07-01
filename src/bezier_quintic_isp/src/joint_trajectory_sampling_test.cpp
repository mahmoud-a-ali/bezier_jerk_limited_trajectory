#include "ros/ros.h"
#include "std_msgs/String.h"
#include <trajectory_interface/quintic_spline_segment.h>
#include <trajectory_interface/trajectory_interface.h>
#include "trajectory_msgs/JointTrajectory.h"


#include<normal_toppra_traj_instant_3.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


using namespace trajectory_interface;

// Floating-point value comparison threshold
const double EPS = 1e-9;

typedef QuinticSplineSegment<double> Segment;
typedef typename Segment::State State;
typedef typename Segment::Time  Time;
typedef std::vector<Segment>    Trajectory;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_trajectory_sampling_test");
  ros::NodeHandle nh;

  //============ read trajectory ==========
  trajectory_msgs::JointTrajectory  traj;
  traj = generate_traj();
  int n_jts = traj.joint_names.size();
  int n_pts = traj.points.size();
  ROS_INFO_STREAM("#joints= " << n_jts << "  &&  #points= " << n_pts );


  // extract pos, vel ,acc
  State states[n_pts];
  Time times[n_pts];
  Trajectory trajectory;

  // considering one joint
  int jt = 0; n_jts=1;
  std::string type = "bezier_quintic";
  double t_start =traj.points[0].positions[0];
  times[0] = traj.points[0].time_from_start.toSec()/1e9;
  states[0].position.push_back( traj.points[0].positions[jt]  );
  states[0].velocity.push_back( traj.points[0].velocities[jt]);
  states[0].acceleration.push_back( traj.points[0].accelerations[jt] );
  for(int jt=0; jt<n_jts; jt++){
      for(int pt=1; pt<n_pts; pt++){
          states[pt].position.push_back( traj.points[pt].positions[jt] );
          states[pt].velocity.push_back( traj.points[pt].velocities[jt] );
          states[pt].acceleration.push_back( traj.points[pt].accelerations[jt] );
          times[pt] = traj.points[pt].time_from_start.toSec()/1e9; //already converted in stored traj
          trajectory.push_back(Segment(times[pt-1], states[pt-1], times[pt], states[pt])  );
      }
  }

  ROS_INFO(" trajeectory is generated");
  std::vector<double> sampled_time, pos, vel, acc, jrk ;

  int frq=125;
  Time time =0.0;
  std::vector<State> STATE;
  while ( time < times[n_pts-1] ){
      State state;
      sample(trajectory, time, state);  //same sample function
      STATE.push_back(state);

      ROS_INFO_STREAM("time: "<< time <<" && state is: " << state.position[0]);

      sampled_time.push_back(time);
      pos.push_back(state.position[0]);
      vel.push_back(state.velocity[0]);
      acc.push_back(state.acceleration[0]);


      time  = time + double(1.0/frq);
  }




  plt::figure(1);
  plt::subplot(2, 2, 1);
  plt::named_plot( "pos",sampled_time, pos); //plt::named_plot( "waypoints", times, states[0].position, "r*");
  plt::title("pos"); plt::grid(true);  plt::legend();
  plt::show();



  return 0;
}
