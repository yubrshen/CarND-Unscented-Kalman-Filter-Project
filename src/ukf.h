#ifndef UKF_H
#define UKF_H

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
public:

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  ///* Augmented sigma points matrix
  MatrixXd Xsig_aug_;

  ///* for predicted sigma points mapped to Radar measurement space
  MatrixXd Zsig_;

  ///* the mapped state in Radar measurement space
  VectorXd z_pred_;

  ///* for the calculation of Radar measurement update
  MatrixXd S_radar_;

  ///* the covariance matrix of Radar measurement
  MatrixXd R_radar_;

  ///* for laser measurement
  MatrixXd H_laser_;

  ///* the covariance matrix for laser measurements
  MatrixXd R_laser_;

  ///* time when the state is true, in us
  long long time_us_; // Not sure what it is for.
  long previous_timestamp_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_ ;

  ///* Weights of sigma points
  double weight_0_; // the weight for sigma point zero
  double weight_; // the weight for the rest of the sigma points

  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Number of sigma points
  int n_sigma_;

  ///* Radar measurement dimension
  int n_z_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* precomputed constant
  double lambda_plus_n_aug;

  ///* the current NIS for radar
  double NIS_radar_;

  ///* the current NIS for laser
  double NIS_laser_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  // Check if the measurement is of acceptable qualify
  bool GoodMeasurement(const MeasurementPackage &measurement_pack);

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Prediction(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLaser(MeasurementPackage meas_package);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(MeasurementPackage meas_package);

  // Called by Prediction for the final computation of the prediction of state, and covariance
  // update x_, and P_
  void PredictMeanAndCovariance();

  // Trasform the augmented sigma points by the process
  void SigmaPointPrediction(double delta_t);

  // Compute the augmented sigmal points
  void AugmentedSigmaPoints();

  // Transform the internal state to be comparable to radar measurement
  void PredictRadarMeasurement();

  // Update state and covariance from radar measurement
  void UpdateStateFromRadar(MeasurementPackage& meas_package);

};

#endif /* UKF_H */
