#include "ukf.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  // use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  // use_radar_ = true;

  sensor_switches_[MeasurementPackage::LASER] = true;
  sensor_switches_[MeasurementPackage::RADAR] = true;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.5; // The original 30 may be too high,
  // std_a_ is computed of the std of the measured rho_dot, which should be a reasonable approximation.

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = M_PI/3;  // The original 30 may be too high
  // std_yawdd set to about 20th of 2*N_PI, about 20 seconds to reach to the speed of turning a full circle in one seconds.
  // It's suggested in the Slack channel to be M_PI/3 with good performance.

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;

  /**
  DONE:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = n_x_ + 2;
  n_sigma_ = 2*n_aug_ + 1;

  n_z_ = 3;

  // initial state vector
  x_ = VectorXd(n_x_); x_.fill(0.0);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_ <<
    1,  0,  0,    0,    0,
    0,  1,  0,    0,    0,
    0,  0,  1,    0,    0,
    0,  0,  0,    1,    0,
    0,  0,  0,    0,    1;      // initial guess

  z_pred_ = VectorXd(n_z_); z_pred_.fill(0.0);

  lambda_ = 3 - n_aug_; // lambda_ + n_aug_ = 3 constant!
  lambda_plus_n_aug = lambda_ + n_aug_;

  weight_0_ = lambda_/lambda_plus_n_aug;
  weight_ = 0.5/lambda_plus_n_aug;

  Xsig_pred_ = MatrixXd(n_x_, n_sigma_); Xsig_pred_.fill(0.0);

  Xsig_aug_ = MatrixXd(n_aug_, n_sigma_); Xsig_aug_.fill(0.0);
  Zsig_ = MatrixXd(n_z_, n_sigma_); Zsig_.fill(0.0);
  z_pred_ = VectorXd(n_z_); z_pred_.fill(0.0);

  S_radar_ = MatrixXd(n_z_, n_z_); S_radar_.fill(0.0);
  R_radar_ = MatrixXd(n_z_, n_z_); R_radar_.fill(0.0);
  R_radar_(0,0) = std_radr_*std_radr_;
  R_radar_(1,1) = std_radphi_*std_radphi_;
  R_radar_(2,2) = std_radrd_*std_radrd_;

  H_laser_ = MatrixXd(2, n_x_);
  H_laser_ <<
    1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0, 0.0;

  R_laser_ = MatrixXd(2, 2);
  //measurement covariance matrix - laser
  R_laser_ <<
    std_laspx_*std_laspx_, 0,
    0,                     std_laspy_*std_laspy_;

  NIS_radar_ = 0; // initial guess
  NIS_laser_ = 0; // initial guess

  not_yet_predicted_ = true;
}

UKF::~UKF() {}

const double SMALL_MEASUREMENT = 0.01; // 0.01m = 1 cm. sqrt(0.01^2 + 0.01^2) = ca. 1.4 cm

inline float SquaredDistance(const float& px, const float& py) {
  return px*px + py*py;
}

bool UKF::GoodMeasurement(const MeasurementPackage &measurement_pack) {
  switch (measurement_pack.sensor_type_) {
  case MeasurementPackage::RADAR:
    if (SMALL_MEASUREMENT < measurement_pack.raw_measurements_[0]) {
      return true;
    }
    break;
  case MeasurementPackage::LASER:
    if (SMALL_MEASUREMENT < SquaredDistance(measurement_pack.raw_measurements_[0],
                                 measurement_pack.raw_measurements_[1])) {
      return true;
    }
    break;
  }
  return false; // reject not qualified data
}

MeasurementPackage FixedMeasurement(const MeasurementPackage &measurement_pack) {
  MeasurementPackage copied;
  switch (measurement_pack.sensor_type_) {
  case MeasurementPackage::RADAR:
    if (measurement_pack.raw_measurements_[0] < SMALL_MEASUREMENT) {
      copied = MeasurementPackage(measurement_pack);
      copied.raw_measurements_[0] = SMALL_MEASUREMENT;
      return copied;
    }
    break;
  case MeasurementPackage::LASER:
    if (SquaredDistance(measurement_pack.raw_measurements_[0],
                        measurement_pack.raw_measurements_[1]) < SMALL_MEASUREMENT) {
      copied = MeasurementPackage(measurement_pack);
      copied.raw_measurements_[0] = SMALL_MEASUREMENT;
      copied.raw_measurements_[1] = SMALL_MEASUREMENT;
      return copied;
    }
    break;
  }
  return measurement_pack;
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  DONE:
  Complete this function! Make sure you switch between lidar and radar measurements.
  */
  // pre-condition: the measurement is acceptable for processing
  MeasurementPackage measurement_pack = FixedMeasurement(meas_package);
  // Initialization
  if (!is_initialized_) {
    /**
       * Initialize the state x_ with the first measurement.
       * Remember: you'll need to convert radar from polar to cartesian coordinates.
       */
    // first measurement
    cout << "UKF: " << endl;
    previous_timestamp_ = measurement_pack.timestamp_;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      x_.fill(0.0);
      x_[0] = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      x_[1] = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      x_[2] = measurement_pack.raw_measurements_[2]; // rho_dot may be approximation to the velocity
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0, 0;
    }
    is_initialized_ = true;
    return; // no more point to process this initial measurement
  }
  if (sensor_switches_[measurement_pack.sensor_type_]) { // add guard to only proceed with the enabled sensor
    // Prediction
    double delta_t = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
    delta_t_ = delta_t;
    previous_timestamp_ = measurement_pack.timestamp_;
    while (0.1 < delta_t) {
      const double dt = 0.05;
      Prediction(dt);
      delta_t -= dt;
    }
    Prediction(delta_t);
    // Use the sensor type to perform the update step.
    // Update the state and covariance matrices.
    switch (measurement_pack.sensor_type_) {
    case MeasurementPackage::RADAR:
      UpdateRadar(measurement_pack);
      break;
    case MeasurementPackage::LASER:
      UpdateLaser(measurement_pack);
      break;
    }
    // print the output
    cout << "x_ = \n" << x_ << endl;
    cout << "P_ = \n" << P_ << endl;
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  DONE:
  Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  AugmentedSigmaPoints();
  SigmaPointPrediction(delta_t);
  PredictMeanAndCovariance();
}

// inline
double AngleNormalize(double angle_in) {
  double angle = angle_in;
  if (M_PI < fabs(angle)) {
    double full_circle = 2*M_PI;
    angle = fmod(angle, full_circle);
    while (angle < -M_PI) angle += full_circle;
    while (M_PI < angle) angle -= full_circle;
  }
  return angle;
}

double AngleNormalize_alternative(double angle_in) {
  double angle = angle_in;
  if (M_PI < fabs(angle)) {
    angle = atan2(sin(angle), cos(angle));
  }
  return angle;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
MatrixXd UKF::S_laser_inversed() {
  return (H_laser_ * P_ * H_laser_.transpose() + R_laser_).inverse();
}

void UKF::UpdateLaser(MeasurementPackage meas_package) {
  /**
  DONE:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd y = meas_package.raw_measurements_ - H_laser_*x_;

  MatrixXd Ht = H_laser_.transpose();
  //MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S_laser_inversed(); // S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  x_(3) = AngleNormalize(x_(3));
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;

  VectorXd y_new = meas_package.raw_measurements_ - H_laser_*x_;
  MatrixXd Si_new = S_laser_inversed(); // (H_laser_ * P_ * Ht + R_laser_).inverse();
  NIS_laser_ = y_new.transpose()*Si_new*y_new;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  DONE:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  PredictRadarMeasurement();
  UpdateStateFromRadar(meas_package);
}

// Called by Prediction for the final computation of the prediction of state, and covariance
// update x_, and P_


void UKF::PredictMeanAndCovariance() {
  // assume sigma points transformed by the process are updated.
  // cout << "Xsig_pred_: \n" << Xsig_pred_ << endl;
  x_.fill(0);
  for (int i=1; i < n_sigma_; i++) {
    x_ = x_ + Xsig_pred_.col(i);
  }
  x_ = weight_0_*Xsig_pred_.col(0) + weight_*x_;
  x_(3) = AngleNormalize(x_(3));

  //predict state covariance matrix
  VectorXd D;
  P_.fill(0);
  for (int i=1; i < n_sigma_; i++){
    D = Xsig_pred_.col(i) - x_;
    D(3) = AngleNormalize(D(3));

    P_ = P_ + D*D.transpose();
  }
  D = Xsig_pred_.col(0) - x_;
  D(3) = AngleNormalize(D(3));

  P_ = weight_0_*(D*D.transpose()) + weight_*P_;
}

// Trasform the augmented sigma points by the process
void UKF::SigmaPointPrediction(double delta_t) {
  //predict sigma points
  //avoid division by zero
  // cout << "Xsig_aug_:\n" << Xsig_aug_ << endl;

  double delta_t_sq = delta_t*delta_t;
  double px=0, py=0, v=0, yaw=0, yaw_dot=0, nu_a=0, nu_yaw_dot_dot=0;
  double c1=0, c2=0, c3=0;
  for (int i = 0; i < n_sigma_; i++) {
    px             = Xsig_aug_(0, i);
    py             = Xsig_aug_(1, i);
    v              = Xsig_aug_(2, i);
    yaw            = Xsig_aug_(3, i);
    yaw_dot        = Xsig_aug_(4, i);
    nu_a           = Xsig_aug_(5, i);
    nu_yaw_dot_dot = Xsig_aug_(6, i);

    c3 = 0.5*delta_t_sq*nu_a;
    c1 = v*delta_t + c3;
    if (fabs(yaw_dot) < 0.0001) {
      Xsig_pred_(0, i) = px + cos(yaw)*c1;
      Xsig_pred_(1, i) = py + sin(yaw)*c1;
    } else {
      c2 = (v/yaw_dot);
      Xsig_pred_(0, i) = px + c2*(sin(yaw + yaw_dot*delta_t) - sin(yaw)) + c3*cos(yaw);
      Xsig_pred_(1, i) = py + c2*(-cos(yaw + yaw_dot*delta_t) + cos(yaw)) + c3*sin(yaw);
    }
    Xsig_pred_(2, i) = v + delta_t*nu_a;
    Xsig_pred_(3, i) = yaw + yaw_dot*delta_t + 0.5*delta_t_sq*nu_yaw_dot_dot;
    Xsig_pred_(3, i) = AngleNormalize(Xsig_pred_(3, i));

    Xsig_pred_(4, i) = yaw_dot + delta_t*nu_yaw_dot_dot;
  }
}

// Compute the augmented sigmal points
void UKF::AugmentedSigmaPoints() {
  //create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug_); x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;

  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_); P_aug.fill(0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;

  // cout << "P_aug:\n" << P_aug << endl;
  //create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  // cout << "square root matrix:\n" << A << endl;

  //create augmented sigma points
  double c = sqrt(lambda_plus_n_aug);
  Xsig_aug_.col(0) = x_aug; // the previous mean
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i+1)        = x_aug + c*A.col(i);
    Xsig_aug_(3, i+1)         = AngleNormalize(Xsig_aug_(3, i+1));

    Xsig_aug_.col(i+1+n_aug_) = x_aug - c*A.col(i);
    Xsig_aug_(3, i+1+n_aug_)  = AngleNormalize(Xsig_aug_(3, i+1+n_aug_));
  }
}

MatrixXd toRadarSpace(MatrixXd cartesian) {
  double p_x=0, p_y=0, v=0, yaw=0, rho=0, phi=0;
  p_x = cartesian(0);
  p_y = cartesian(1);
  v = cartesian(2);
  yaw = cartesian(3);
  rho = sqrt(p_x*p_x + p_y*p_y);
  VectorXd radar(3);
  radar(0) = rho;
  phi = atan2(p_y, p_x);
  radar(1) = phi;
  if (fabs(rho) < 0.0001) {
    radar(2) = 0;
  } else {
    radar(2) = (p_x*cos(yaw) + p_y*sin(yaw))*v/rho;
  }
  return radar;
}

void UKF::PredictRadarMeasurement() {
  //transform sigma points into radar measurement space
  for (int i=0; i < n_sigma_; i++) {
    Zsig_.col(i) = toRadarSpace(Xsig_pred_.col(i));
  }
  //calculate mean predicted measurement
  z_pred_.fill(0);
  for (int i=1; i < n_sigma_; i++) {
    z_pred_ = z_pred_ + Zsig_.col(i);
  }
  z_pred_ = weight_0_*Zsig_.col(0) + weight_*z_pred_;
  z_pred_(1) = AngleNormalize(z_pred_(1));

  //calculate measurement covariance matrix S_radar_
  VectorXd d;
  S_radar_.fill(0);
  for (int i=1; i < n_sigma_; i++){
    d = Zsig_.col(i) - z_pred_;
    d(1) = AngleNormalize(d(1));

    S_radar_ = S_radar_ + d*d.transpose();
  }
  d = Zsig_.col(0) - z_pred_;
  d(1) = AngleNormalize(d(1));

  S_radar_ = weight_0_*(d*d.transpose()) + weight_*S_radar_;

  // add R, measurement covariance
  S_radar_ = S_radar_ + R_radar_;
}

void UKF::UpdateStateFromRadar(MeasurementPackage& meas_package) {
  // get the measurement
  VectorXd z = meas_package.raw_measurements_;
  //calculate cross correlation matrix
  VectorXd d_x, d_z;
  MatrixXd Tc = MatrixXd(n_x_, n_z_); Tc.fill(0.0);

  for (int i=1; i < n_sigma_; i++){
    d_x = Xsig_pred_.col(i) - x_;
    d_x(3) = AngleNormalize(d_x(3));

    d_z = Zsig_.col(i) - z_pred_;
    d_z(1) = AngleNormalize(d_z(1));

    Tc = Tc + d_x*d_z.transpose();
  }
  d_x = Xsig_pred_.col(0) - x_;
  d_x(3) = AngleNormalize(d_x(3));

  d_z = Zsig_.col(0) - z_pred_;
  d_z(1) = AngleNormalize(d_z(1));

  Tc = weight_0_*d_x*d_z.transpose() + weight_*Tc;

  //calculate Kalman gain K;
  MatrixXd Si = S_radar_.inverse();
  MatrixXd K = Tc*Si;

  //update state mean and covariance matrix
  VectorXd z_diff = (z - z_pred_);
  z_diff(1) = AngleNormalize(z_diff(1));

  x_ = x_ + K*z_diff;
  x_(3) = AngleNormalize(x_(3));

  P_ = P_ - K*S_radar_*K.transpose();

  VectorXd z_diff_new = z - toRadarSpace(x_);
  z_diff_new(1) = AngleNormalize(z_diff_new(1));

  NIS_radar_ = z_diff_new.transpose()*Si*z_diff_new;
}

// NEXT: debug the code
// investigate on how to reduce RMSE
