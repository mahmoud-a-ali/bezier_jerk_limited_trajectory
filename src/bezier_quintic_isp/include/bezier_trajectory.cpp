#include "bezier_trajectory.h"
//#include <python2.7/Python.h>
//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

bezier_trajectory::bezier_trajectory()
{

}

void bezier_trajectory::init(std::vector<double> waypoints_pos, std::vector<double> waypoints_vel,
                             std::vector<double> waypoints_acc, std::vector<double> waypoints_durations, double start_time){
    std::cout<<"initiate bezier_joint_trajectory ... "<<std::endl;

    if (waypoints_pos.empty() ) // || waypoints_vel.empty() || waypoints_acc.empty()  )
      throw(std::invalid_argument("bezier_joint_trajectory can't be constructed: waypoints_pos are empty "));
    else
        n_points_ = waypoints_pos.size();

    if (!waypoints_pos.empty() && !waypoints_vel.empty() && (waypoints_pos.size() != waypoints_vel.size() ) )
      throw(std::invalid_argument("bezier_joint_trajectory can't be constructed: provided positions and velocities have different sizes"));

    if (!waypoints_pos.empty() && !waypoints_acc.empty() && (waypoints_pos.size() != waypoints_acc.size() ) )
      throw(std::invalid_argument("bezier_joint_trajectory can't be constructed: provided positions and accelerations have different sizes"));

    if (!waypoints_pos.empty() && !waypoints_durations.empty() && (waypoints_pos.size()-1 != waypoints_durations.size() ) )
      throw(std::invalid_argument("bezier_joint_trajectory can't be constructed: #durations should be equal to #positions-1 "));


    if ( waypoints_vel.empty()){
      std::cout<<"Warn: no velocities are provided, trajectory will stop at each waypoint" << std::endl;
      for (int i=0; i< n_points_; i++)
          velocities_.push_back( 0);
    }

    if (waypoints_acc.empty()){
        std::cout<<"Warn: no accelerations are provided, accerlartion will be zero at each waypoint" << std::endl;
        for (int i=0; i< n_points_; i++)
            accelerations_.push_back( 0);
      }

    if (waypoints_durations.empty()){
        std::cout<<"Warn: no duration are provided, min duration which grantee all limits will be considered for each segment" << std::endl;
        for (int i=0; i< n_points_-1 ; i++)
            durations_.push_back(0.01); //could result in nan
    }

    if ( !waypoints_pos.empty() )
        positions_ = waypoints_pos;
    if ( !waypoints_vel.empty() )
        velocities_ = waypoints_vel;
    if ( !waypoints_acc.empty() )
        accelerations_ = waypoints_acc;

    if ( !waypoints_durations.empty() ){
        durations_ = waypoints_durations;
        for (int i=0; i< n_points_-1; i++){
            if(durations_[i] < 0.01 ){
                durations_[i] = 0.01;
                std::cout<<"Warn: segs "<<i << " has duration equal to zero, it will initialized with 0.01 as min" << std::endl;
            }
        }
    }

    t_start_ = start_time;
    times_.resize( n_points_);
    times_[0]= t_start_;

    for (int i=0; i< n_points_-1; i++)
        times_[i+1] = times_[i] + durations_[i];

    std::cout<<"bezier_joint_trajectory has been initialized ... "<<std::endl;

}


void bezier_trajectory::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("segment can't be constructed: absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
}




std::vector<double>  bezier_trajectory::update_waypoints_times( ){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return update_waypoints_times(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}




std::vector<double> bezier_trajectory::update_waypoints_times( double max_pos, double max_vel, double max_acc, double max_jrk ){
    set_absolute_limits( max_pos, max_vel, max_acc, max_jrk);
    std::vector<double> start_state, end_state;
    start_state.resize(3); end_state.resize(3);

    for(int seg= 0; seg<n_points_-1 ; seg++){
        std::cout<<">>>> update time: seg"<< seg<< std::endl;
        start_state[0]= positions_[seg] ;     end_state[0]= positions_[seg+1] ;
        start_state[1]= velocities_[seg] ;    end_state[1]= velocities_[seg+1] ;
        start_state[2]= accelerations_[seg];  end_state[2]= accelerations_[seg+1] ;

        bezier_quintic_segment traj_seg( times_[seg], durations_[seg], start_state, end_state );
        traj_seg.set_absolute_limits(max_pos_, max_vel_, max_acc_, max_jrk_ );
        durations_[seg] = traj_seg.update_duration( );
        times_[seg+1] = times_[seg]+ durations_[seg]; //update end_time whih start time of nxt seg
//        traj_seg.print_attributes();

    }
    return times_;


}




// finds in which segment time instant t belons to
int bezier_trajectory::find_segment (double t){
    // segment changes from 0 to n-2 which mean n-1 seg
    int seg = 0;
    if(t<= times_[0]) //less than tstart
        return seg;
    else if(t>= times_[ n_points_-2 ] ) //if t is greater than the starting time of the last segment
        return n_points_-2;             // seg= 0 to n_points_-2 ==> n_points_-1 segment
    else {
        for (int i=0; i< n_points_-2 ; i++) { // between tstart and tend
            if(t>= times_[i] && t< times_[i+1])
                seg = i;
        }
        return seg;
    }
}



void bezier_trajectory::sample_bezier_trajectory(double t, std::vector<double> &state){
    state.resize(4);
    std::vector<double> start_state, end_state;
    start_state.resize(3); end_state.resize(3);
    bezier_quintic_segment segment;
    segment.set_absolute_limits(max_pos_, max_vel_, max_acc_, max_jrk_ );
    int seg= find_segment(t);
//    std::cout<<"seg: "<< seg <<std::endl;
    start_state[0]= positions_[seg] ;     end_state[0]= positions_[seg+1] ;
    start_state[1]= velocities_[seg] ;    end_state[1]= velocities_[seg+1] ;
    start_state[2]= accelerations_[seg];  end_state[2]= accelerations_[seg+1] ;
    segment.init( times_[seg], durations_[seg], start_state, end_state );
    segment.compute_coef();
    segment.sample_segment(t, state);
 }


void bezier_trajectory::trajectory_states(const double frq, std::vector<double> &T_vec, std::vector<double> &POS, std::vector<double> &VEL, std::vector<double> &ACC, std::vector<double> &JRK ){
    double t= t_start_;
    std::vector<double> state;
    while(t<times_.back()){
        sample_bezier_trajectory(t, state);
        T_vec.push_back(t);
        POS.push_back(state[0]);
        VEL.push_back(state[1]);
        ACC.push_back(state[2]);
        JRK.push_back(state[3]);
        t+=1/frq;
    }
}


void bezier_trajectory::print_attributes(){
    std::cout<<"###limits: \n" <<"pos: " <<max_pos_ <<",  vel: " <<max_vel_<<", acc: " <<max_acc_<<",  jrk: " <<max_jrk_<<std::endl;

    std::cout<< "###positions_:  "<<std::endl;
    for (int i=0; i<positions_.size(); i++)
        std::cout<< positions_[i]<<std::endl;

    std::cout<< "###velocities_:  "<<std::endl;
    for (int i=0; i<velocities_.size(); i++)
        std::cout<< velocities_[i]<<std::endl;

    std::cout<< "###accelerations_:  "<<std::endl;
    for (int i=0; i<accelerations_.size(); i++)
        std::cout<< accelerations_[i]<<std::endl;

    std::cout<< "###jerks_:  "<<std::endl;
    for (int i=0; i<jerks_.size(); i++)
        std::cout<< jerks_[i]<<std::endl;

    std::cout<< "###durations_:  "<<std::endl;
    for (int i=0; i<durations_.size(); i++)
        std::cout<< durations_[i]<<std::endl;

    std::cout<< "###times_:  "<<std::endl;
    for (int i=0; i<times_.size(); i++)
        std::cout<< times_[i]<<std::endl;

}














//===========================================================================
// here bezier_robot_trajectory
//============================================================================
void bezier_robot_trajectory::init(std::vector<std::vector<double>> waypoints_pos, std::vector<std::vector<double>> waypoints_vel,
                                   std::vector<std::vector<double>> waypoints_acc, std::vector<double> waypoints_durations, double start_time){

    if (waypoints_pos.empty() ) // || waypoints_vel.empty() || waypoints_acc.empty()  )
      throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: waypoints_pos are empty "));
    else{
        n_joints_ = waypoints_pos.size();
        n_points_ = waypoints_pos[0].size();
        std::cout<<"initiate bezier_robot_trajectory: n_joints_ =  "<< n_joints_ <<"  && n_points_ = "<< n_points_ <<std::endl;
    }


    if (!waypoints_pos.empty() && !waypoints_vel.empty() && (waypoints_pos.size() != waypoints_vel.size() ) )
      throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: positions and velocities have different joints sizes"));

    if (!waypoints_pos.empty() && !waypoints_acc.empty() && (waypoints_pos.size() != waypoints_acc.size() ) )
      throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: positions and accelerations have different joints sizes"));

//    if (!waypoints_pos.empty() && !waypoints_durations.empty() && (waypoints_pos.size()-1 != waypoints_durations.size() ) )
//      throw(std::invalid_argument("trajectory can't be constructed: #durations should be equal to #positions-1 "));


    if ( waypoints_vel.empty()){
      std::cout<<"Warn: no velocities are provided, trajectory will stop at each waypoint" << std::endl;
      velocities_.resize(n_joints_);
      for (int jt=0; jt< n_joints_; jt++)
          for (int pt=0; pt< n_points_; pt++)
              velocities_[jt].push_back( 0);
    }

    if (waypoints_acc.empty()){
        std::cout<<"Warn: no accelerations are provided, accerlartion will be zero at each waypoint" << std::endl;
        accelerations_.resize(n_joints_);
        for (int jt=0; jt< n_joints_; jt++)
            for (int pt=0; pt< n_points_; pt++)
                accelerations_[jt].push_back( 0);
      }

    if (waypoints_durations.empty()){
        std::cout<<"Warn: no duration are provided, min duration which grantee all limits will be considered for each segment" << std::endl;
        for (int pt=0; pt< n_points_-1 ; pt++)
            sync_durations_.push_back(0.01); //could result in nan
    }

    if ( !waypoints_pos.empty() )
        positions_ = waypoints_pos;
    if ( !waypoints_vel.empty() )
        velocities_ = waypoints_vel;
    if ( !waypoints_acc.empty() )
        accelerations_ = waypoints_acc;

    if ( !waypoints_durations.empty() ){
        sync_durations_ = waypoints_durations;
        for (int pt=0; pt< n_points_-1; pt++){
            if(sync_durations_[pt] < 0.01 ){
                sync_durations_[pt] = 0.01;
                std::cout<<"Warn: seg "<<pt << " has duration equal to zero, it will be initialized with 0.01 as min" << std::endl;
            }
        }
    }

    times_.resize(n_joints_);
    durations_.resize(n_joints_);
    t_start_ = start_time;
    sync_times_.resize( n_points_);
    sync_times_[0]= t_start_;
    for (int pt=0; pt< n_points_-1; pt++)
        sync_times_[pt+1] = sync_times_[pt] + sync_durations_[pt];

    joint_trajectories_.resize( n_joints_);
    for (int jt=0; jt< n_joints_; jt++)
        joint_trajectories_[jt].init(positions_[jt], velocities_[jt], accelerations_[jt], sync_durations_, t_start_);

    std::cout<<"bezier_robot_trajectory has been initialized ... " << std::endl;

}



void bezier_robot_trajectory::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
    for (int jt=0; jt< n_joints_; jt++)
        joint_trajectories_[jt].set_absolute_limits(max_pos_, max_vel_, max_acc_, max_jrk_);
}




std::vector<double> bezier_robot_trajectory::update_waypoints_times(){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return update_waypoints_times(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}



std::vector<double> bezier_robot_trajectory::update_waypoints_times(double max_pos, double max_vel, double max_acc, double max_jrk){
    set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
    for (int jt=0; jt< n_joints_; jt++) {
        std::cout<<"\n====================== update to time: joint_traj: "<< jt << " =======================" << std::endl;
        times_[jt] = joint_trajectories_[jt].update_waypoints_times();
    }
    compute_durations_from_times(times_, durations_);
    generate_sync_durations( durations_, sync_durations_);
    compute_times_from_durations(t_start_, sync_durations_, sync_times_);

    std::cout<<"\n======================== synchronize all joitns ==========================" << std::endl;
    for (int jt=0; jt< n_joints_; jt++) {
        std::cout<<"sync_durations_.size(): "<< sync_durations_.size() << std::endl;
        joint_trajectories_[jt].init(positions_[jt], velocities_[jt], accelerations_[jt], sync_durations_, t_start_);
        joint_trajectories_[jt].update_waypoints_times();
    }

    return sync_times_;
}



void bezier_robot_trajectory::sample_bezier_robot_trajectory(double t, std::vector<double> &POS, std::vector<double> &VEL,
                                                                       std::vector<double> &ACC, std::vector<double> &JRK){
    POS.resize(n_joints_);  VEL.resize(n_joints_);
    ACC.resize(n_joints_);  JRK.resize(n_joints_);
    std::vector<double> state;
    for (int jt=0; jt< n_joints_; jt++) {
        joint_trajectories_[jt].sample_bezier_trajectory(t, state);
        POS[jt] = state[0];
        VEL[jt] = state[1];
        ACC[jt] = state[2];
        JRK[jt] = state[3];
    }
 }


void bezier_robot_trajectory::init_from_trajectory_msg( trajectory_msgs::JointTrajectory msg ){
    // extract Positions, Velocities ,Accelerations, Times from that trajectory and store them in std::vectors
//    if(msg.joint_names.empty())
//        throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: no joints has been found"));
//    if(msg.points[0].positions.empty())
//        throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: no points has been found"));

//    n_joints_ = msg.joint_names.size();
//    n_points_ = msg.points.size();

//    for(int pt=0; pt<n_joints_; pt++)
//        if(msg.points[pt].positions.empty())
//            throw(std::invalid_argument("bezier_robot_trajectory can't be constructed: different  #joitns for each way points"));

//    positions_.resize(n_joints_);
//    velocities_.resize(n_joints_);
//    accelerations_.resize(n_joints_);

//    times_.resize(n_joints_);
//    durations_.resize(n_joints_);

//    sync_times_.resize(n_points_);
//    sync_durations_.resize(n_points_-1);

//    t_start_ = msg.points[0].positions[0];

//    for(int jt=0; jt<n_joints_; jt++){
//        for(int pt=0; pt<n_points_; pt++){
//            positions_[jt].push_back( msg.points[pt].positions[jt] );
//            velocities_[jt].push_back( msg.points[pt].velocities[jt] );
//            accelerations_[jt].push_back( msg.points[pt].accelerations[jt] );
//            times_[jt].push_back( msg.points[pt].time_from_start.toSec()/1e9); //already converted in stored traj
//            sync_times_.push_back( msg.points[pt].time_from_start.toSec()/1e9);
//        }
//    }
//    bezier_robot_trajectory::compute_durations_from_times( times_, durations_ );
////    bezier_robot_trajectory::compute_durations_from_times( sync_times_, sync_durations_ );
//    init(positions_, velocities_, accelerations_, durations_[0], t_start_);


    int n_jts = msg.joint_names.size();
    int n_pts = msg.points.size();


    // extract Positions, Velocities ,Accelerations, Times from that trajectory and store them in std::vectors
    std::vector< std::vector<double> > Pos_jt_wpt, Vel_jt_wpt, ACC_jt_wpt, Times_jt_wpt, durations;
    Pos_jt_wpt.resize(n_jts);
    Vel_jt_wpt.resize(n_jts);
    ACC_jt_wpt.resize(n_jts);
    Times_jt_wpt.resize(n_jts);
    durations.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            Pos_jt_wpt[jt].push_back( msg.points[pt].positions[jt] );
            Vel_jt_wpt[jt].push_back( msg.points[pt].velocities[jt] );
            ACC_jt_wpt[jt].push_back( msg.points[pt].accelerations[jt] );
//            Times_jt_wpt[jt].push_back(0);
            Times_jt_wpt[jt].push_back( msg.points[pt].time_from_start.toSec()/1e9); //already converted in stored traj
        }
    }
    double t_start =Times_jt_wpt[0][0];
    std::vector<double> required_duration;
    compute_durations_from_times( Times_jt_wpt[0], required_duration );
    init(Pos_jt_wpt , Vel_jt_wpt, ACC_jt_wpt, durations[0], t_start );



}












void bezier_robot_trajectory::generate_sync_durations( std::vector< std::vector<double> > traj_durations, std::vector<double> &sync_durations){
    int n_jts= traj_durations.size();
    int n_pts= traj_durations[0].size() +1;
    sync_durations.resize(n_pts-1);

    //takes max durations for each segment over different joints/traajectories, create synchronised durations
    for(int seg=0; seg<n_pts-1 ; seg++){
       sync_durations[seg] = traj_durations[0][seg];
       for(int jt=1; jt<n_jts ; jt++){
           if( sync_durations[seg] < traj_durations[jt][seg]  )
               sync_durations[seg]= traj_durations[jt][seg] ;
       }
    }
}


void bezier_robot_trajectory::compute_durations_from_times( std::vector<double> traj_times, std::vector<double>  &traj_durations){
    //create trajectories durations from times
    int n_pts=traj_times.size();
    traj_durations.resize( n_pts -1);
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[seg] = traj_times[seg+1] - traj_times[seg];
}


void bezier_robot_trajectory::compute_times_from_durations( double t_start, std::vector<double>  traj_durations,  std::vector<double>  &traj_times ){
    //create trajectories times from durations
    int n_pts=traj_durations.size()+1;
    traj_times.resize( n_pts );
    traj_times[0]= t_start;

    for (int pt=0; pt< n_pts-1; pt++)
            traj_times[pt+1] = traj_times[pt] + traj_durations[pt]; // t= last_time + duration of that segment

}


void bezier_robot_trajectory::compute_durations_from_times(std::vector< std::vector<double> > traj_times, std::vector< std::vector<double> > &traj_durations){
    //create trajectories durations from times
    int n_jts=traj_times.size();
    int n_pts=traj_times[0].size();
    traj_durations.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[jt].push_back( traj_times[jt][seg+1] - traj_times[jt][seg]);
}


void bezier_robot_trajectory::compute_times_from_durations(double t_start, std::vector<std::vector<double>> traj_durations, std::vector< std::vector<double> > &traj_times ){
    //create trajectories times from durations
    int n_jts=traj_durations.size();
    int n_pts=traj_durations[0].size()+1;
    traj_times.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++){
        traj_times[jt][0]= t_start;
        for (int pt=0; pt< n_pts-1; pt++)
            traj_times[jt][pt+1] = traj_times[jt][pt] + traj_durations[jt][pt]; // t= last_time + duration of that segment
     }
}


void bezier_robot_trajectory::print_attributes(){
    std::cout<<"============================ attributes bezier_robot_trajectory ============================"<<std::endl;
    for (int jt=0; jt< n_joints_; jt++) {
        std::cout<<"--------------------------- attributes of joint "<< jt<<"-----------------------------"<<std::endl;
        joint_trajectories_[jt].print_attributes();
    }

}








