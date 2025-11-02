#include "allegro_utils/kalman_filter_joint.hpp"

using namespace kalman_filter_joint;

KalmanFilterJoint::KalmanFilterJoint(int n, double sigma_u, Eigen::MatrixXd Rk){
    hat_x = Eigen::VectorXd::Zero(3*n);
    Pk = Eigen::MatrixXd::Identity(3*n,3*n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
    Eigen::Matrix3d A;
    A << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix3d A2;
    A2 << 0, 0, 1,
          0, 0, 0,
          0, 0, 0;
    Eigen::RowVector3d C;
    C << 1, 0, 0;

    Fk = Eigen::KroneckerProduct(I, A);
    F2k = Eigen::KroneckerProduct(I, A2);
    Hk = Eigen::KroneckerProduct(I, C);

    this->sigma_u = sigma_u;
    this->Rk = Rk;
}
KalmanFilterJoint::KalmanFilterJoint(int n, Eigen::MatrixXd Qk, Eigen::MatrixXd Rk){
    hat_x = Eigen::VectorXd::Zero(3*n);
    Pk = Eigen::MatrixXd::Identity(3*n,3*n);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
    Eigen::Matrix3d A;
    A << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
    Eigen::Matrix3d A2;
    A2 << 0, 0, 1,
          0, 0, 0,
          0, 0, 0;
    Eigen::RowVector3d C;
    C << 1, 0, 0;

    Fk = Eigen::KroneckerProduct(I, A);
    F2k = Eigen::KroneckerProduct(I, A2);
    Hk = Eigen::KroneckerProduct(I, C);

    this->Qk = Qk;
    this->Rk = Rk;
}
void KalmanFilterJoint::prediction(double dt){
    int n = hat_x.size();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n,n);
    Eigen::MatrixXd Fd = Eigen::MatrixXd::Identity(n,n) + dt * Fk + dt*dt/2 * F2k;
    hat_x = Fd * hat_x;

    Eigen::Vector3d G;
    G << 1/2*dt*dt, dt, 1;

    //std::cout << "Q:    " << Eigen::KroneckerProduct(I, G*G.transpose()).rows()    << "x"  << Eigen::KroneckerProduct(I, G*G.transpose()).cols() << std::endl;
    //std::cout << "Pk:    " << Pk.rows()    << "x"  << Pk.cols() << std::endl;
    Eigen::MatrixXd Qk = (sigma_u*sigma_u)* Eigen::KroneckerProduct(Eigen::MatrixXd::Identity(n/3,n/3), G*G.transpose());

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
