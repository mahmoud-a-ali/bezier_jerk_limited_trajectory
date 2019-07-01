
    //#include <ros.h>
    #include <trajectory_msgs/JointTrajectory.h>
    #include <trajectory_msgs/JointTrajectoryPoint.h>
    //this script writes cpp file to instantiate JointTrajectory msg from yaml file"
    
trajectory_msgs::JointTrajectory generate_traj( ) 
 {
trajectory_msgs::JointTrajectory traj; 
   traj.joint_names.push_back("joint_1");
   traj.joint_names.push_back("joint_2");
   traj.joint_names.push_back("joint_3");
   traj.joint_names.push_back("joint_4");
   traj.joint_names.push_back("joint_5");
   traj.joint_names.push_back("joint_6");

        trajectory_msgs::JointTrajectoryPoint pt;  
        // copy info from Python pt to cpp pt
        
pt.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0 );
traj.points.push_back(pt);

pt.positions = { 0.20, 0.20, 0.20, 0.20, 0.20, 0.20};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0.3e9 );
traj.points.push_back(pt);



pt.positions = { 0.20, 0.60, 0.20, 0.60, 0.20, 0.60};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0.5e9 );
traj.points.push_back(pt);



pt.positions = {0.60, 0.10, 0.60, 0.10, 0.60, 0.10};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0.6e9 );
traj.points.push_back(pt);


pt.positions = {01.90, 0.10, 01.90, 0.10, 01.90, 0.10};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0.9e9 );
traj.points.push_back(pt);



pt.positions = {0.10, 01.90,0.10, 01.90, 0.10, 01.90};
pt.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
pt.time_from_start = ros::Duration( 0.95e9 );
traj.points.push_back(pt);





          return traj;
        }
        
