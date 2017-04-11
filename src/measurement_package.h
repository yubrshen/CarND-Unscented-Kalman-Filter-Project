#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  MeasurementPackage() {}
  MeasurementPackage(const MeasurementPackage& rhs) {
    timestamp_ = rhs.timestamp_;
    sensor_type_ = rhs.sensor_type_;
    raw_measurements_ = rhs.raw_measurements_;
  }
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
