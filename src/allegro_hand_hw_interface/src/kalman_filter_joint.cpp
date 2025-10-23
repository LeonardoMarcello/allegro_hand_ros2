#include "allegro_hand_hw_interface/kalman_filter_joint.hpp"

using namespace kalman_filter_joint;

KalmanFilterJoint::KalmanFilterJoint(int n, Eigen::MatrixXd Qk, Eigen::MatrixXd Rk){
    hat_x = Eigen::VectorXd::Zero(3*n);
    Pk = Eigen::MatrixXd::Identity(3*n,3*n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
    Eigen::Matrix3d A;
    A << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::RowVector3d C;
    C << 1, 0, 0;

    Fk = Eigen::KroneckerProduct(I, A);
    Hk = Eigen::KroneckerProduct(I, C);


    this->Qk = Qk;
    this->Rk = Rk;
}

void KalmanFilterJoint::prediction(double dt){
    int n = hat_x.size();
    Eigen::MatrixXd Fd = Eigen::MatrixXd::Identity(n, n) + dt * Fk;
    hat_x = Fd * hat_x;
    Pk = Fd * Pk * Fd.transpose() + Qk;
}
void KalmanFilterJoint::update(Eigen::VectorXd z){
    int n = hat_x.size();
    Eigen::VectorXd yk = z - Hk * hat_x;
    Eigen::MatrixXd Sk = Hk * Pk * Hk.transpose() + Rk;
    Eigen::MatrixXd Kk = Pk * Hk.transpose() * Sk.inverse();
    hat_x = hat_x + Kk * yk;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);
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

void KalmanFilterJoint::get_q(double *q){
    int n = hat_x.size()/3;
    if (sizeof(q) == n) {
        for (int i = 0; i < n; ++i){
            q[i] = hat_x(3 * i);
        }
    }else{
		std::cout<<"in get_q: invalid dimensions of vector q\n";
    }
}
void KalmanFilterJoint::get_dq(double *dq){
    int n = hat_x.size()/3;
    if (sizeof(dq) == n) {
        for (int i = 0; i < n; ++i){
            dq[i] = hat_x(3 * i + 1);
        }
    }else{
		std::cout<<"in get_dq: invalid dimensions of vector dq\n";
    }
}
void KalmanFilterJoint::get_ddq(double *ddq){
    int n = hat_x.size()/3;
    if (sizeof(ddq) == n) {
        for (int i = 0; i < n; ++i){
            ddq[i] = hat_x(3 * i + 2);
        }
    }else{
		std::cout<<"in get_ddq: invalid dimensions of vector ddq\n";
    }
}
