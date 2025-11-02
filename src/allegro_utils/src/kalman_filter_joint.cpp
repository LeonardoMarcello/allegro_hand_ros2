#include "allegro_utils/kalman_filter_joint.hpp"

using namespace kalman_filter_joint;

KalmanFilterJoint::KalmanFilterJoint(double sigma_u, double sigma_r){
    hat_x = Eigen::VectorXd::Zero(3);
    Pk = Eigen::MatrixXd::Identity(3,3);

    Ak.resize(3,3); A2k.resize(3,3); Hk.resize(1,3);
    Ak << 0, 1, 0,
          0, 0, 1,
          0, 0, 0;
    A2k << 0, 0, 1,
           0, 0, 0,
           0, 0, 0;
    Hk << 1, 0, 0;

    this->sigma_u = sigma_u;
    this->Rk = sigma_r * Eigen::MatrixXd::Identity(1,1);
}
void KalmanFilterJoint::prediction(double dt){
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd Fd = I + dt*Ak + dt*dt/2*A2k;
    hat_x = Fd * hat_x;

    Eigen::Vector3d G;
    G << 1/2*dt*dt, dt, 1;

    Eigen::MatrixXd Qk = (sigma_u*sigma_u)*G*G.transpose();

    Pk = Fd * Pk * Fd.transpose() + Qk;
}

void KalmanFilterJoint::update(Eigen::VectorXd z){
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    Eigen::VectorXd yk = z - Hk * hat_x;
    Eigen::MatrixXd Sk = Hk * Pk * Hk.transpose() + Rk;
    Eigen::MatrixXd Kk = Pk * Hk.transpose() * Sk.inverse();
    hat_x = hat_x + Kk * yk;

    Pk = (I - Kk * Hk) * Pk * (I - Kk * Hk).transpose() + Kk * Rk * Kk.transpose();
}
void KalmanFilterJoint::get_q(Eigen::VectorXd &q){
    int n = hat_x.size()/3;
    if (q.size() == n) {
        for (int i = 0; i < n; ++i){
            q(i) = hat_x(3 * i);
        }
    }else{
		std::cout<<"in get_q: invalid dimensions of vector q\n";
    }
}
void KalmanFilterJoint::get_dq(Eigen::VectorXd &dq){
    int n = hat_x.size()/3;
    if (dq.size() == n) {
        for (int i = 0; i < n; ++i){
            dq(i) = hat_x(3 * i + 1);
        }
    }else{
		std::cout<<"in get_dq: invalid dimensions of vector dq\n";
    }
}
void KalmanFilterJoint::get_ddq(Eigen::VectorXd &ddq){
    int n = hat_x.size()/3;
    if (ddq.size() == n) {
        for (int i = 0; i < n; ++i){
            ddq(i) = hat_x(3 * i + 2);
        }
    }else{
		std::cout<<"in get_ddq: invalid dimensions of vector ddq\n";
    }
}


void KalmanFilterJoint::update(double z){
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(3,3);
    Eigen::VectorXd yk = z*Eigen::MatrixXd::Identity(1,1) - Hk * hat_x;
    Eigen::MatrixXd Sk = Hk * Pk * Hk.transpose() + Rk;
    Eigen::MatrixXd Kk = Pk * Hk.transpose() * Sk.inverse();
    hat_x = hat_x + Kk * yk;

    Pk = (I - Kk * Hk) * Pk * (I - Kk * Hk).transpose() + Kk * Rk * Kk.transpose();
}
void KalmanFilterJoint::get_q(double &q){
    q = hat_x(0);
}
void KalmanFilterJoint::get_dq(double &dq){
    dq = hat_x(1);
}
void KalmanFilterJoint::get_ddq(double &ddq){
    ddq = hat_x(2);
}
