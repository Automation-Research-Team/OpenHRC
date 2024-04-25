#include "ohrc_control/feedback_controller.hpp"

void FeedbackController::initInterface(){
  t0_f.resize(3,-1000.0);
}

void FeedbackController::updateTargetPose(KDL::Frame& pose, KDL::Twist& twist) {
  KDL::Frame frame;
  KDL::Twist vel;
  controller->getCartState(frame, vel);

  VectorXd f = controller->getForceEefVec();
  VectorXd v(3);

  
  VectorXd v_force = this->forceFeedbackControl(frame, pose, f);
  VectorXd v_pi = this->PIControl(frame, pose, twist);

  for (int i=0;i<3;i++){
    // if (f.head(3).norm() < 1.0)
    if(std::abs(f[i]) < 1.0)
      v[i] = v_pi[i];
    else{
      v[i] = v_force[i];
      t0_f[i] = ros::Time::now().toSec();
    }
  }


  tf::vectorEigenToKDL(v, twist.vel);
}

//https://qiita.com/Kurounmo/items/70ca355f5be97df1e9da
double pseudoSigmoid(double x, double x0, double w, double y0, double h){
  double y;
  x = (x-x0)/w;
  
  double eps = 1.0e-5;
  if(x < eps)
    y=y0;
  else if(x > 1.0 - eps)
    y=h+y0;
  else{
    double f = std::exp(-1.0/x);
    double g = std::exp(-1.0/(1.0-x));
    y=h*f/(f+g)+y0;
  }
  return y;

}


VectorXd FeedbackController::PIControl(const KDL::Frame& frame, const KDL::Frame& pose, const KDL::Twist& twist) {
  double kp = 1.2, ki = 0.0;//1.0*dt;
  Vector3d p_cur, p_des, v_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);
  tf::vectorKDLToEigen(twist.vel, v_des);

  // double d_gain_x = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[0]) / 1.0));
  // double d_gain_y = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[1]) / 1.0));
  // double d_gain_z = 1.0 - std::max(0., std::min(0.4, std::abs(v_des[2]) / 1.0));

  // MatrixXd D_inv = Vector3d(d_gain_x,d_gain_y,d_gain_z).cwiseInverse().asDiagonal();

  // static Vector3d e_integ = Vector3d::Zero();
  // e_integ += p_des - p_cur;

  // for (int i=0;i<3;i++){
  //   if(std::abs(v_des[i]) < 0.01){
  //     e_integ[i]=0.0;
  //     kp=0.0;
  //   }
  // }


  double a = 0.2;
  double b = 2.0;
  std::vector<double> gain(3);

  for (int i=0;i<3;i++){
    double y0 = a* std::exp(-b*(ros::Time::now().toSec() - t0_f[i])); // 0.0
    gain[i] = pseudoSigmoid(std::abs(v_des[i]),0.0,0.4,y0,2.0);
  }

 
  return v_des + Vector3d(gain.data()).asDiagonal() * kp * (p_des - p_cur);// + ki * e_integ;
}


VectorXd FeedbackController::forceFeedbackControl(const KDL::Frame& frame, const KDL::Frame& pose, const VectorXd f) {
  double kf = 0.001, alpha = 100.0;// estimated force command corresponding to 1 m position error. 
  Vector3d p_cur, p_des;
  tf::vectorKDLToEigen(frame.p, p_cur);
  tf::vectorKDLToEigen(pose.p, p_des);

  return kf * (alpha * (p_des - p_cur) + f.head(3));
}