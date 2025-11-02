#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

namespace kalman_filter_joint{
    class KalmanFilterJoint{

        public:
            KalmanFilterJoint(int n, double sigma_u, Eigen::MatrixXd Rk);
            KalmanFilterJoint(int n, Eigen::MatrixXd Qk, Eigen::MatrixXd Rk);
            KalmanFilterJoint(double sigma_u, double sigma_r);
            ~KalmanFilterJoint() = default;
            void prediction(double dt);
            void update(Eigen::VectorXd z);
            void get_q(Eigen::VectorXd &q);
            void get_dq(Eigen::VectorXd &dq);
            void get_ddq(Eigen::VectorXd &ddq);

        private:

            Eigen::VectorXd hat_x;
            Eigen::MatrixXd Pk;

            Eigen::MatrixXd Fk;
            Eigen::MatrixXd F2k;
            Eigen::MatrixXd Hk;
            
            double sigma_u;
            Eigen::MatrixXd Qk;
            Eigen::MatrixXd Rk;

    }; //class KalmanFilterJoint
} //namespace kalman_filter_joint