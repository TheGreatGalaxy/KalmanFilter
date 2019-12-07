//
// Created by abang on 17-10-26.
//

#include "ukf.h"
#include <iostream>

using namespace std;
// using Eigen::MatrixXd;
// using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
    // if this is false, laser measurements will be ignored (except during init)
    use_laser_ = true;

    // if this is false, radar measurements will be ignored (except during init)
    use_radar_ = false;
    is_initialized_ = false;
    // n_x_ = 5;
    n_x_ = 4;
    // n_aug_ = 7;
    n_aug_ = 6;
    lambda_ = 3 - n_aug_;
    // initial state vector
    x_ = Eigen::VectorXd(n_x_);

    // initial covariance matrix
    P_ = Eigen::MatrixXd(n_x_, n_x_);

    // Process noise standard deviation longitudinal acceleration in m/s^2
    std_a_ = 2.0;

    // Process noise standard deviation yaw acceleration in rad/s^2
    std_yawdd_ = 0.7;

    // Laser measurement noise standard deviation position1 in m
    std_laspx_ = 0.15;

    // Laser measurement noise standard deviation position2 in m
    std_laspy_ = 0.15;

    // // Radar measurement noise standard deviation radius in m
    // std_radr_ = 0.3;

    // // Radar measurement noise standard deviation angle in rad
    // std_radphi_ = 0.03;

    // // Radar measurement noise standard deviation radius change in m/s
    // std_radrd_ = 0.3;

    // P_ << 1, 0, 0, 0, 0,
    //         0, 1, 0, 0, 0,
    //         0, 0, 1, 0, 0,
    //         0, 0, 0, 1, 0,
            // 0, 0, 0, 0, 1;
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1;
    x_.fill(0.0);
    Xsig_pred_ = Eigen::MatrixXd(n_x_, 2 * n_aug_ + 1);

    weights_ = Eigen::VectorXd(2 * n_aug_ + 1);
    weights_[0] = lambda_/ (lambda_ + n_aug_);
    for(int i=1; i < (2*n_aug_+1); i++){
        weights_[i] = 1/(2 * (lambda_ + n_aug_));
    }

    R_laser_ = Eigen::MatrixXd(2, 2);
    R_laser_ << std_laspx_*std_laspx_, 0,
            0, std_laspy_*std_laspy_;

    // R_radar_ = MatrixXd(3, 3);
    // R_radar_ << std_radr_*std_radr_, 0, 0,
    //         0, std_radphi_*std_radphi_, 0,
    //         0, 0, std_radrd_*std_radrd_;

}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
    /**
    TODO:

    Complete this function! Make sure you switch between lidar and radar
    measurements.
    */
    if(!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER){
        return;
    }
    if(!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR){
        return;
    }
    if (!is_initialized_) {
        // first measurement
        x_.fill(0.0);

        if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
            x_[0] = meas_package.raw_measurements_[0];
            x_[1] = meas_package.raw_measurements_[1];
        }
        // } else {
        //     float rho = meas_package.raw_measurements_[0];
        //     float phi = meas_package.raw_measurements_[1];
        //     float rho_dot = meas_package.raw_measurements_[2];
        //     x_[0] = rho * cos(phi);
        //     x_[1] = rho * sin(phi);

        // }
        time_us_ = meas_package.timestamp_;
        is_initialized_ = true;
        return;
    }
    double delta_t =(meas_package.timestamp_ - time_us_) /  1000000.0;
    time_us_ = meas_package.timestamp_;
    Prediction(delta_t);

    // if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
        // UpdateRadar(meas_package);
    // } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package);
    // }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);
    Xsig_aug.fill(0.0);
    AugmentedSigmaPoints(&Xsig_aug);
    SigmaPointPrediction(Xsig_aug, delta_t);
    PredictMeanAndCovariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {

    Eigen::VectorXd z = meas_package.raw_measurements_;
    long n_z = z.rows();

    Eigen::VectorXd z_pred = Eigen::VectorXd(n_z);
    Eigen::MatrixXd S_out = Eigen::MatrixXd(n_z, n_z);
    Eigen::MatrixXd Zsig = Eigen::MatrixXd(n_z, 2 * n_aug_ + 1);

    PredictLaserMeasurement(z_pred, S_out, Zsig, n_z);

    UpdateState(z, z_pred, S_out, Zsig, n_z);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
// void UKF::UpdateRadar(MeasurementPackage meas_package) {


//     VectorXd z = meas_package.raw_measurements_;
//     long n_z = z.rows();

//     VectorXd z_pred = VectorXd(n_z);
//     MatrixXd S_out = MatrixXd(n_z, n_z);
//     MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

//     PredictRadarMeasurement(z_pred, S_out, Zsig, n_z);

//     UpdateState(z, z_pred, S_out, Zsig, n_z);
// }

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd *Xsig_out) {

    //create augmented mean vector
    Eigen::VectorXd x_aug = Eigen::VectorXd(n_aug_);

    //create augmented state covariance
    Eigen::MatrixXd P_aug = Eigen::MatrixXd(n_aug_, n_aug_);

    //create sigma point matrix
    Eigen::MatrixXd Xsig_aug = Eigen::MatrixXd(n_aug_, 2 * n_aug_ + 1);

    //create augmented mean state
    //create augmented covariance matrix
    //create square root matrix
    //create augmented sigma points
    x_aug.head(n_x_) = x_;
    x_aug(4) = 0;
    x_aug(5) = 0;

    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x_, n_x_) = P_;
    P_aug(4,4) = std_a_*std_a_;
    P_aug(5,5) = std_a_*std_a_;

    Eigen::MatrixXd A = P_aug.llt().matrixL();

    //create augmented sigma points
    Xsig_aug.col(0)  = x_aug;
    for (int i = 0; i< n_aug_; i++)
    {
        Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * A.col(i);
        Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * A.col(i);
    }

    //write result
    *Xsig_out = Xsig_aug;
}

void UKF::SigmaPointPrediction(Eigen::MatrixXd &Xsig_aug, double delta_t) {

    for(int i =0; i < (2 * n_aug_ + 1); i++){
        Eigen::VectorXd input_x = Xsig_aug.col(i);
        float px = input_x[0];
        float py = input_x[1];
        float vx = input_x[2];
        float vy = input_x[3];
        float ax = input_x[4];
        float ay = input_x[5];        
        // float v = input_x[2];
        // float psi = input_x[3];
        // float psi_dot = input_x[4];
        // float mu_a = input_x[5];
        // float mu_psi_dot_dot = input_x[6];

        Eigen::VectorXd term2 = Eigen::VectorXd(n_x_);
        Eigen::VectorXd term3 = Eigen::VectorXd(n_x_);

        Eigen::VectorXd result = Eigen::VectorXd(n_x_);
        // if(psi_dot < 0.001){
            // term2 << v * cos(psi) * delta_t, v * sin(psi) * delta_t, 0, psi_dot * delta_t, 0;
            // term3 << 0.5 * delta_t*delta_t * cos(psi) * mu_a,
            //         0.5 * delta_t*delta_t * sin(psi) * mu_a,
            //         delta_t * mu_a,
            //         0.5 * delta_t*delta_t * mu_psi_dot_dot,
            //         delta_t * mu_psi_dot_dot;
            // result = Xsig_aug.col(i).head(5) + term2 + term3;
        // } else{
        //     term2 << (v/psi_dot) * (sin(psi + psi_dot * delta_t) - sin(psi)),
        //             (v/psi_dot) * (-cos(psi + psi_dot * delta_t) + cos(psi)),
        //             0,
        //             psi_dot * delta_t,
        //             0;

        //     term3 << 0.5 * delta_t*delta_t * cos(psi) * mu_a,
        //             0.5 * delta_t*delta_t * sin(psi) * mu_a,
        //             delta_t * mu_a,
        //             0.5 * delta_t*delta_t * mu_psi_dot_dot,
        //             delta_t * mu_psi_dot_dot;
        //     result = Xsig_aug.col(i).head(5) + term2 + term3;
        // }
        term2 << vx*delta_t + 0.5*ax*delta_t*delta_t, vy*delta_t + 0.5*ay*delta_t*delta_t, ax*delta_t, ay*delta_t;
        result = Xsig_aug.col(i).head(n_x_) + term2;
        Xsig_pred_.col(i) = result;
    }
}

void UKF::PredictMeanAndCovariance() {
    x_.fill(0.0);
    for(int i=0; i<2*n_aug_+1; i++){
        x_ = x_+ weights_[i] * Xsig_pred_.col(i);
    }

    P_.fill(0.0);
    for(int i=0;  i<2*n_aug_+1; i++){
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
        // while (x_diff[3]> M_PI)
        //     x_diff[3] -= 2.*M_PI;
        // while (x_diff[3] <-M_PI)
        //     x_diff[3]+=2.*M_PI;
        P_ = P_ + weights_[i] * x_diff * x_diff.transpose();
    }
}

void UKF::PredictLaserMeasurement(Eigen::VectorXd &z_pred, Eigen::MatrixXd &S, Eigen::MatrixXd &Zsig, long n_z) {
    for(int i=0; i < 2*n_aug_+1; i++){
        float px = Xsig_pred_.col(i)[0];
        float py = Xsig_pred_.col(i)[1];
        Eigen::VectorXd temp = Eigen::VectorXd(n_z);
        temp << px, py;
        Zsig.col(i) = temp;
    }

    z_pred.fill(0.0);
    for(int i=0; i < 2*n_aug_+1; i++){
        z_pred = z_pred + weights_[i] * Zsig.col(i);
    }

    S.fill(0.0);
    for(int i=0; i < 2*n_aug_+1; i++){
        //residual
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

        S = S + weights_[i] * z_diff * z_diff.transpose();
    }
    S = S + R_laser_;
}


// void UKF::PredictRadarMeasurement(VectorXd &z_pred, MatrixXd &S, MatrixXd &Zsig, long n_z) {
//     for(int i=0; i < 2*n_aug_+1; i++){
//         float px = Xsig_pred_.col(i)[0];
//         float py = Xsig_pred_.col(i)[1];
//         float v = Xsig_pred_.col(i)[2];
//         float psi = Xsig_pred_.col(i)[3];
//         float psi_dot = Xsig_pred_.col(i)[4];

//         float temp = px * px + py * py;
//         float rho = sqrt(temp);
//         float phi = atan2(py, px);
//         float rho_dot;
//         if(fabs(rho) < 0.0001){
//             rho_dot = 0;
//         } else{
//             rho_dot =(px * cos(psi) * v + py * sin(psi) * v)/rho;
//         }

//         VectorXd temp1 = VectorXd(3);
//         temp1 << rho, phi, rho_dot;
//         Zsig.col(i) = temp1;
//     }

//     z_pred.fill(0.0);
//     for(int i=0; i < 2*n_aug_+1; i++){
//         z_pred = z_pred + weights_[i] * Zsig.col(i);
//     }

//     S.fill(0.0);
//     for(int i=0; i < 2*n_aug_+1; i++){
//         //residual
//         VectorXd z_diff = Zsig.col(i) - z_pred;

//         //angle normalization
//         while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
//         while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
//         S = S + weights_[i] * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose();
//     }
//     S = S + R_radar_;
// }


void UKF::UpdateState(Eigen::VectorXd &z, Eigen::VectorXd &z_pred, Eigen::MatrixXd &S, Eigen::MatrixXd &Zsig, long n_z) {

    //create matrix for cross correlation Tc
    Eigen::MatrixXd Tc = Eigen::MatrixXd(n_x_, n_z);

    //calculate cross correlation matrix
    //calculate Kalman gain K;
    //update state mean and covariance matrix

    Tc.fill(0.0);
    for(int i=0; i < 2*n_aug_+1; i++){
        Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;

        // while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
        // while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

        //residual
        Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;
        // if(n_z == 3){
        //     //angle normalization
        //     while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
        //     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
        // }
        Tc = Tc + weights_[i] * x_diff * z_diff.transpose();
    }

    Eigen::MatrixXd K = Eigen::MatrixXd(n_x_, n_z);
    K = Tc * S.inverse();

    Eigen::VectorXd y = z - z_pred;
    //angle normalization
    // if(n_z == 3){
    //     while (y(1)> M_PI) y(1)-=2.*M_PI;
    //     while (y(1)<-M_PI) y(1)+=2.*M_PI;
    // }
    x_ = x_ + K * y;
    P_ = P_ - K * S * K.transpose();
}

