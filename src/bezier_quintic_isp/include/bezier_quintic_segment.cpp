#include "bezier_quintic_segment.h"

bezier_quintic_segment::bezier_quintic_segment()
{

}


void bezier_quintic_segment::init(double start_time, double duration, std::vector<double> start_state, std::vector<double> end_state){
    if (start_time < 0)
      throw(std::invalid_argument("segment can't be constructed: start_time should be positive "));

    if ( duration < 0)
      throw(std::invalid_argument("segment can't be constructed: duration should be positive."));


    if (start_state.empty() || end_state.empty())
      throw(std::invalid_argument("segment can't be constructed: start & end points  can't be empty."));

    if (start_state.size() != 3)
      throw(std::invalid_argument("segment can't be constructed: startpoint should have 3 values (pos, vel, acc)"));

    if (end_state.size() != 3)
      throw(std::invalid_argument("segment can't be constructed: endpoint should have 3 values (pos, vel, acc)"));

    t_start_ = start_time;
    set_duration_( duration);
//    t_end_   = start_time + duration;
    start_state_ = start_state;
    end_state_ = end_state;
    coef_.resize(6);
    iter_=0;
}


void bezier_quintic_segment::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("segment can't be constructed: absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
}


double  bezier_quintic_segment::update_duration( ){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return update_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}

double  bezier_quintic_segment::update_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
//    std::cout<<"update duration 1"<< std::endl;
    set_absolute_limits( max_pos,  max_vel,  max_acc,  max_jrk);
    compute_coef();
    compute_maxmin_times();
//    std::cout<<"update duration 2"<< std::endl;

    if( check_jerk_limit() && check_acc_limit() && check_vel_limit() ){
        std::cout<<"requisted time already grantee jrk, pos, vel limits" <<std::endl;
        // check for monotic condition
        return duration_;
    }
    else{
        while (!check_jerk_limit() || !check_acc_limit() || !check_vel_limit()  ){
//            t_end_ += 0.01;
            set_duration_( duration_ + 0.01);
            compute_coef();
            compute_maxmin_times();
            if( check_jerk_limit() && check_acc_limit() && check_vel_limit() ){
//                duration_ = t_end_ - t_start_;
                return duration_;
//                std::cout<<"requisted time doesn't grantee jrk, pos, vel limits"<<std::endl;
//                std::cout<< "new duration is:  "<< duration_ <<"  new end_time" << t_end_;
            }
            iter_ ++;
        }
    }
}



void bezier_quintic_segment::compute_coef(){
    coef_.resize(6);
    double t1=t_start_, t2=t_end_;
    double p0=start_state_[0], v0=start_state_[1], a0=start_state_[2], pn=end_state_[0], vn=end_state_[1], an=end_state_[2];
    coef_[0] = -p0/pow((t1 - t2), 5) ;
    coef_[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1 - t2),5));
    coef_[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1 - t2), 5));
    coef_[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1 - t2), 5));
    coef_[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1 - t2), 5));
    coef_[5] = -pn/pow((t1 - t2), 5);
}


bool bezier_quintic_segment::check_jerk_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t1, t2, t_maxjrk_, t_maxacc_, t_minacc_};
    double jrk =0;
     for(auto t : tm){
        jrk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
//        std::cout<<"check_jrk: "<< t <<"   check_jrk: "<< jrk<<std::endl;
        if(fabs(jrk) > max_jrk_)
           return false;
     }
     return true;
}


bool bezier_quintic_segment::check_acc_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t1, t2, t_maxjrk_, t_maxacc_, t_minacc_};
    double acc =0;
     for(auto t : tm){
         acc = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
//         std::cout<<"check_acc: "<< t<<"     check_acc: "<< acc<<std::endl;
         if(fabs(acc) > max_acc_)
           return false;
     }
     return true;
}



bool bezier_quintic_segment::check_vel_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t1, t2, t_maxjrk_, t_maxacc_, t_minacc_};
    double vel =0;
    for(auto t: tm){
         vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    //    std::cout<<"check_vel: "<< t <<"     check_vel: "<< vel<<std::endl;
        if(fabs(vel) > max_vel_)
            return false;
     }
     return true;
}


void bezier_quintic_segment::sample_segment(double t, std::vector<double> &state) {
    double t1=t_start_, t2=t_end_;
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    state.resize(4); //pos, vel, acc, jrk
    state[0] = P5*pow( (t-t1), 5) - P0*pow( (t-t2), 5) + 5*P1*(t - t1)*pow( (t-t2), 4) - P4*(5*t - 5*t2)*pow( (t-t1), 4) - 10*P2*pow( (t-t1), 2)*pow( (t-t2), 3) + 10*P3*pow( (t-t1), 3)*pow( (t-t2), 2);
    state[1] = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    state[2] = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
    state[3] = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
}



void bezier_quintic_segment::segment_states(const double frq, std::vector<double> &T_vec, std::vector<double> &POS, std::vector<double> &VEL, std::vector<double> &ACC, std::vector<double> &JRK ){
    double t=t_start_;
    std::vector<double> state;
    while(t<t_end_){
        sample_segment(t, state);
        T_vec.push_back(t);
        POS.push_back(state[0]);
        VEL.push_back(state[1]);
        ACC.push_back(state[2]);
        JRK.push_back(state[3]);
        t+=1/frq;
    }
}


void bezier_quintic_segment::print_attributes(){
    std::cout<<"======================== segment attributes ========================="<<std::endl;
    std::cout<<"###coef_: \n" <<"P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;
    std::cout<<"###limits: \n" <<"pos: " <<max_pos_ <<",  vel: " <<max_vel_<<", acc: " <<max_acc_<<",  jrk: " <<max_jrk_<<std::endl;
    std::cout<<"###times: \n" <<"t_start: " <<t_start_ <<",  t_end: " <<t_end_<<", duration: " <<duration_<<std::endl;
    std::cout<<"###maxmin_times: \n" <<"t_minacc_: " <<t_minacc_ <<",  t_maxjrk_: " <<t_maxjrk_<<", t_maxacc_: " <<t_maxacc_<<std::endl;
    std::cout<<"###start_state: \n" <<"pos0: " <<start_state_[0] <<",  vel0: " <<start_state_[1]<<", acc0: " <<start_state_[2]<<std::endl;
    std::cout<<"###start_state: \n" <<"pos1: " <<end_state_[0] <<",  vel1: " <<end_state_[1]<<", acc1: " <<end_state_[2]<<std::endl;
    std::cout<<"###iterations: "<<iter_<<"     ###monotonic: "<<check_monotonic()<<std::endl;
    std::cout<<"=========================================================================="<<std::endl;

}






















// calculate min  duration
double bezier_quintic_segment::compute_min_duration( ){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return compute_min_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}

double bezier_quintic_segment::compute_min_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
    duration_ = 0.01; //starting with one time sample
    return update_duration();
}


bool bezier_quintic_segment::check_generic_transition(){
    if( fabs(start_state_[1])<0.001 && fabs(start_state_[2])<0.001 && fabs(end_state_[1])<0.001 && fabs(end_state_[2])<0.001 )
        return false;
    else
        return true;
}

// calculate max  duration
double bezier_quintic_segment::compute_max_duration(){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return compute_max_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}

double bezier_quintic_segment::compute_max_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
    if( !check_generic_transition()){
        std::cout<<"segment transition is from zero state(vel, acc) to zero state(vel , acc), so ih has no max duration (consider 10e5)"<<std::endl;
        return 100000;
    }
    //    std::cout<<"calculate max duration*/..."<<std::endl;
    set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
    duration_ = compute_min_duration();
    compute_coef();
    compute_maxmin_times();
    std::cout<<"==> calculate max duration, starting with min duration..."<< duration_<<std::endl;
//    std::cout<<"check_monotonic() ..."<< check_monotonic() <<std::endl;
    while(check_monotonic()){
        t_end_ += 0.01;
        compute_coef();
        compute_maxmin_times();
    }
    // you choose to set duration to its max or not
    duration_ = t_end_ - t_start_;
    return t_end_ - t_start_;
}



void calculate_min_rrot_cubic_eq(double r1, double r2){
    double a, b, c;
    double t = a*a + b*c + b*b;
    double t2 = a*c;


}



void bezier_quintic_segment::compute_maxmin_times(){
//    std::cout<<">>compute_maxmin_times: t_star, t_end " << t_start_ << ",  "<< t_end_<< std::endl;
//    std::cout<<">>compute_maxmin_times: coef_: \n" <<"P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;
    double A=0, B=0, C=0, D=0, a=0, b=0, c=0, d=0, disc=0;
    double t1=t_start_, t2=t_end_;
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    t_maxjrk_ = (120*P0*t2 - 120*P1*t1 - 480*P1*t2 + 480*P2*t1 + 720*P2*t2 - 720*P3*t1 - 480*P3*t2 + 480*P4*t1 + 120*P4*t2 - 120*P5*t1)/(120*P0 - 600*P1 + 1200*P2 - 1200*P3 + 600*P4 - 120*P5);
    //  std::cout<<"t0_snp: "<< t0_snp<< std::endl;

    // old way without considering all possible conditions
     A=  180*P1 -  60*P0 - 180*P2 + 60*P3;
     B= -60*P2  + 180*P3 - 180*P4 + 60*P5;
     C=  120*P1 - 360*P2 + 360*P3 - 120*P4;
     a= A+B+C;
     b= - (2*t2*A + 2*t1*B + t1*C + t2*C);
     c= (A*t2*t2 + B*t1*t1 + C*t1*t2);
     disc= b*b - 4*a*c;
    t_minacc_ = (-b -sqrt(disc) ) / (2*a);
    t_maxacc_ = (-b +sqrt(disc) ) / (2*a);







//    // consider jrk is quadratic:
//    A=  180*P1 -  60*P0 - 180*P2 + 60*P3;
//    B= -60*P2  + 180*P3 - 180*P4 + 60*P5;
//    C=  120*P1 - 360*P2 + 360*P3 - 120*P4;
//    a= A+B+C;
//    b= - (2*t2*A + 2*t1*B + t1*C + t2*C);
//    c= (A*t2*t2 + B*t1*t1 + C*t1*t2);
//    disc= b*b - 4*a*c;
//    t_minacc_ = (-b -sqrt(disc) ) / (2*a);
//    t_maxacc_ = (-b +sqrt(disc) ) / (2*a);

//    if(fabs(a)<0.00000001) { // case that jerk is linear check if it pass through zero
//        std::cout<<"linear jerk case, duration: "<<duration_ <<std::endl;
//        t_maxjrk_ = fabs(-c/b); // so here when jrk cross zero it gives max_min point of acc
//        //  find max and min vel by making acc=0, and start and end time
//            A=  40*P1 -  20*P0 - 20*P2 ;
//            B= 20*P3  - 40*P4 + 20*P5;
//            C=  60*P1 - 120*P2 + 60*P3 ;
//            D=  -60*P2 + 120*P3 - 60*P4 ;
//            a= (A+B+C+D);
//            b= -(3*t2*A + 3*t1*B + (2*t2+t1)*C + (2*t1+t2)*D );
//            c= (3*pow(t2, 2)*A + 3*pow(t1, 2)*B + (pow(t2, 2)+2*t1*t2)*C +  (pow(t1, 2)+2*t1*t2)*D );
//            d= (pow(t2,3)*A + pow(t1,3)*B +  t1*pow(t2, 2)*C  + pow(t1, 2)*t2*D   );
//            //here a=0 as well
//            disc = c*c - 4*b*d;
//            t_minacc_ = (-c- sqrt(disc) )/ (2*b);
//            t_maxacc_ = (-c+ sqrt(disc) )/ (2*b);
//            std::cout<<" times are: "<< t_maxjrk_<<",  "<< t_maxacc_ <<",  "<< t_minacc_ << std::endl;
//    }
//    else {
//         t_maxjrk_ = (120*P0*t2 - 120*P1*t1 - 480*P1*t2 + 480*P2*t1 + 720*P2*t2 - 720*P3*t1 - 480*P3*t2 + 480*P4*t1 + 120*P4*t2 - 120*P5*t1)/(120*P0 - 600*P1 + 1200*P2 - 1200*P3 + 600*P4 - 120*P5);

//    }



//    if( t_maxjrk_ > t_end_ || t_maxjrk_<t_start_ ) {
//        std::cout<<"coditions linear jrk: "<< t_maxjrk_ << std::endl;
//        t_maxjrk_ = t_maxacc_;
//    }
//     else
//        t_maxjrk_ = t_maxjrk_;



//    if( t_maxjrk_<t_start_)
//        t_maxjrk_ = t_start_;

//    if( t_maxjrk_>t_end_)
//        t_maxjrk_ = t_end_;

//    if( t_maxacc_<t_start_)
//        t_maxacc_ = t_start_;

//    if( t_maxacc_>t_end_)
//        t_maxacc_ = t_end_;

//    if( t_minacc_<t_start_)
//        t_minacc_ = t_start_;

//    if( t_minacc_<t_start_)
//        t_minacc_ = t_start_;
//    std::cout<<"#calculate times " << t_start_ <<"  "<< t_end_<<"  "<< t_maxjrk_ << "  "<< t_maxacc_ <<" "<< t_minacc_<< std::endl;

}











bool bezier_quintic_segment::check_monotonic(){
//    if( check_generic_transition() ){
	//make sure of start/end state	
//        std::cout<<" t_star, t_end " << t_start_ << ",  "<< t_end_<< std::endl;
//        std::cout<<"###coef_: \n" <<"P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;

        compute_coef();
//        std::cout<<" t_star, t_end " << t_start_ << ",  "<< t_end_<< std::endl;
//        std::cout<<"###coef_: \n" <<"P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;

        compute_maxmin_times();
        double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
        double t1=t_start_, t2=t_end_;
        double t= t_maxjrk_;
        double reached_vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
        std::cout<<"#check times " << t_maxjrk_ << "  "<< t_maxacc_ <<" "<< t_minacc_<< std::endl;
        if( (reached_vel >=0 && start_state_[1]>=0  &&  end_state_[1]>=0) || (reached_vel <=0 && start_state_[1]<=0  &&  end_state_[1]<=0)  )
            return true;
        else
            return false;
//    }
//    else {
//        std::cout<<" reached_vel at t_maxjrk " << t_maxjrk_ << " is "<< reached_vel<< std::endl;
//        return true;
//    }

}
















//    acc = 40*P1*(t - t2)^3 - 20*P0*(t - t2)^3 - 20*P2*(t - t2)^3 + 20*P3*(t - t1)^3 - 40*P4*(t - t1)^3 + 20*P5*(t - t1)^3 + 60*P1*(t - t1)*(t - t2)^2 - 60*P2*(2*t - 2*t1)*(t - t2)^2 - 30*P2*(2*t - 2*t2)*(t - t1)^2 + 30*P3*(2*t - 2*t1)*(t - t2)^2 + 60*P3*(2*t - 2*t2)*(t - t1)^2 - 12*P4*(5*t - 5*t2)*(t - t1)^2
//    acc1= (A+B+C+D)*t^3  ...
//            - (3*t2*A + 3*t1*B + (2*t2+t1)*C + (2*t1+t2)*D ) *t^2 ...
//            + (3*t2^2*A + 3*t1^2*B + (t2^2+2*t1*t2)*C +  (t1^2+2*t1*t2)*D ) *t ...
//            - (t2^3*A + t1^3*B +  t1*t2^2*C  + t1^2*t2*D   )








