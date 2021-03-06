#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;
    
    previous_timestamp_ = 0;
    
    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;
    
    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
    
    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    
    H_laser_<< 1, 0, 0, 0,
               0, 1, 0, 0;
    
    
    MatrixXd P_ = MatrixXd(4, 4);
    
    P_ <<  1, 0, 0   , 0,
           0, 1, 0   , 0,
           0, 0, 1000, 0,
           0, 0, 0   , 1000;
    
    
//    P_ <<	0.5, 0, 0, 0,
//            0, 0.5, 0, 0,
//            0, 0, 100, 0,
//            0, 0, 0, 100;
    
    MatrixXd F_ = MatrixXd(4, 4);
    F_ << 1, 0, 1, 0,
          0, 1, 0, 1,
          0, 0, 1, 0,
          0, 0, 0, 1;
    

    MatrixXd Q_ = MatrixXd(4, 4);
    
    VectorXd x_ = VectorXd(4);
    x_ = VectorXd(4);
    x_ << 1, 1, 1,1;
    
    ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
    
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "EKF: " << endl;
        
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            
//          Convert radar from polar to cartesian coordinates and initialize state.

            double px, py, vx,vy;
            px = measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]);
            py = measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]);
//            vx = measurement_pack.raw_measurements_[2]*cos(measurement_pack.raw_measurements_[1]);
//            vy = measurement_pack.raw_measurements_[2]*sin(measurement_pack.raw_measurements_[1]);
            //One radar measurement isn't enough to establish speed.
            vx = 0;
            vy = 0;
            
            ekf_.x_ <<px,py,vx,vy;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
//          Initialize state.
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);

//            cout << "First measurement: "<< ekf_.x_<<"\n";
        }
        
        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    
//  Update the state transition matrix F according to the new elapsed time:
    
    double deltaT = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;

    

    ekf_.F_(0, 2) = deltaT;
    ekf_.F_(1, 3) = deltaT;
    //    cout << ekf_.F_;
//  Update the process noise covariance matrix:
    float dt_2 = deltaT * deltaT;
    float dt_3 = dt_2 * deltaT;
    float dt_4 = dt_3 * deltaT;
        
    
    ekf_.Q_ <<  dt_4/4*noise_ax, 0               , dt_3/2*noise_ax, 0,
                0              , dt_4/4*noise_ay , 0              , dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0               , dt_2*noise_ax  , 0,
                0              , dt_3/2*noise_ay , 0              , dt_2*noise_ay;
    
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
//      Update with new radar measurements:
        Tools tools;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
//      Update with new laser measurements:
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
        
    }
    
    // print the output:
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
