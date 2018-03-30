#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  
  // cout << "in UKF constructor start" << endl;
  
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1.0; // was 30

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 10/180.0*M_PI; //= 0.17 was 30
  
  cout << "std_yawdd_" << std_yawdd_ << endl;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
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
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  //set state dimension
  n_x_= 5;
  
  //set augmented dimension
  n_aug_ = 7;
  
  //define spreading parameter
  lambda_ = 3.0 - n_aug_;  
  
  //create vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  weights_.fill(0.0);
  
    // set weights
  weights_(0) = (float) lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  //2n+1 weights
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }

  // cout << "weights_ " << endl <<  weights_ << endl;
  
  //create matrix with predicted sigma points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  is_initialized_ = false;
  
  /**

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  
  // initialize with zero with first measurment would update px and py
  x_ << 0.0,0.0,0.0,0.0,0.0 ;
  
  
  P_ << 0.2, 0.0,  0.0, 0.0, 0.0,
        0.0,  0.2, 0.0, 0.0, 0.0,
        0.0,  0.0, 4.0, 0.0, 0.0,
        0.0,  0.0, 0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 0.0, 0.1;
  // Keeping velocity variance high as already bycycle is moving at speed of 4.8, 
  // higher value of v variance helped to keep intitial nis within limit
  
  // Working  : 0.2, 0.2, 1.0, 1.0, 0.1 std_a_ = 0.2; std_yawdd_ = 0.2;
  
  /*
  P_ <<     0.0043,   -0.0013,    0.0030,   -0.0022,   -0.0020,
          -0.0013,    0.0077,    0.0011,    0.0071,    0.0060,
           0.0030,    0.0011,    0.0054,    0.0007,    0.0008,
          -0.0022,    0.0071,    0.0007,    0.0098,    0.0100,
          -0.0020,    0.0060,    0.0008,    0.0100,    0.0123;
  */    
  // cout << "in UKF constructor end" << endl;
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  
  // cout << "in ProcessMeasurement start" << endl;
  
  if(!is_initialized_){
    time_us_ = meas_package.timestamp_;
    
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR){    
    
      // cout << "in init RADAR start" << endl;
      
      VectorXd x_meaured = meas_package.raw_measurements_;
      float rho = x_meaured[0];
      float phi = x_meaured[1];
      
      float x = rho * cos(phi);
      float y = rho * sin(phi);
      
      x_(0)=x;
      x_(1)=y;
      
      // cout << "in init RADAR end" << endl;
      
    } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      
      // cout << "in init LASER start" << endl;      
      
      VectorXd x_meaured = meas_package.raw_measurements_;
      
      x_(0)=x_meaured[0];
      x_(0)=x_meaured[1];
      
      // cout << "in init LASER end" << endl; 
    
    }
    
    is_initialized_ = true;
    return;    
  }
  
  // cout << "in ProcessMeasurement before prediction start" << endl;
  
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  
  
  // cout << "delta_t : " << delta_t << endl; 
  
  // cout << "===================PREDICTION  START======================" << endl;
  Prediction(delta_t);
  // cout << "===================PREDICTION  END========================" << endl;
  use_radar_=true;
  use_laser_=true;
  
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
    
    if(use_radar_){
      // cout << "===================UpdateRadar  START======================" << endl;  
      UpdateRadar(meas_package);
      time_us_ = meas_package.timestamp_;
      // cout << "===================UpdateRadar  END======================" << endl;
    }
    
  } else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    
    if(use_laser_){
      // cout << "===================UpdateLidar  START======================" << endl;      
      UpdateLidar(meas_package);
      time_us_ = meas_package.timestamp_;
      // cout << "===================UpdateLidar  END======================" << endl;
    }
    
  }
  
  // cout << "in init ProcessMeasurement end" << endl;
  
}

/**
 Generate signma points using augmented covariance matrix
*/
void UKF::generateAugmentedSigmaPoints(MatrixXd* Xsig_aug){ 

  //create augmented mean state vector  
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  
  x_aug.head(5)=x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  // cout << "x_aug : " << endl << x_aug << endl;
 
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_); 
  
  P_aug.fill(0.0);    
  P_aug.topLeftCorner(n_x_,n_x_)=P_;
  P_aug(5,5)=std_a_*std_a_;
  P_aug(6,6)=std_yawdd_*std_yawdd_;

  // cout << "P_aug : " << endl << P_aug << endl;
  
  Xsig_aug->fill(0.0);
  
  MatrixXd A = P_aug.llt().matrixL();
  
  // create augmented sigma points
  Xsig_aug->col(0)=x_aug;
  for(int i=0;i<n_aug_;i++){
     Xsig_aug->col(i+1) = x_aug + sqrt(lambda_+n_aug_)*A.col(i);
     Xsig_aug->col(i+n_aug_+1) = x_aug - sqrt(lambda_+n_aug_)*A.col(i);
  }
  
  // cout << "Xsig_aug : " << endl << Xsig_aug << endl;
}


/**
  Predict new state using signma points and non linear equaltions.
*/
void UKF::predictNewStateUsingSigmaPoints(MatrixXd& Xsig_aug,double delta_t){
  
  // cout << "in Prediction before generating sigma points" << endl;
  // Generate sigma points
  for(int i=0;i<(2*n_aug_ + 1);++i){
    
    VectorXd sigma_point = Xsig_aug.col(i);
    
    double pos_x = sigma_point(0);
    double pos_y = sigma_point(1);
    double vk = sigma_point(2);
    double yaw_k = sigma_point(3);
    double yaw_rate_k = sigma_point(4);
    double v_acc_noise = sigma_point(5);
    double yaw_acce_noise = sigma_point(6);
  
    
    /* Calculate process noise */
    // process noise for postition 1
    // 1/2 * (delta_t)^2 * cos(yaw_angle) * v_acc_noise
    double poistion_noise1= 0.5 *(delta_t*delta_t) * cos(yaw_k) *v_acc_noise;
    // 1/2 * (delta_t)^2 * sin(yaw_angle) * v_acc_noise
    double poistion_noise2= 0.5 *(delta_t*delta_t) * sin(yaw_k) * v_acc_noise;
    // (delta_t) * v_acc_noise
    double velocity_noise= delta_t * v_acc_noise;
    // 1/2 * (delta_t)^2 * yaw_acce_noise
    double yaw_noise= 0.5 *(delta_t*delta_t) * yaw_acce_noise;
    //(delta_t) * yaw_rate_noise
    double yaw_rate_noise= (delta_t) * yaw_acce_noise;     

    double px,py,v,yaw,yaw_rate=0.0;
    
    // state values after deta_t time with addition of noise 
    if(fabs(yaw_rate_k) < 0.001) // yaw is almost zero so avoid devide by zero
    {
      // cout << "estimating px, py using cos and sin" << endl;
      // px = xk + vk*cos(yaw_angle)*delta_t + poistion_noise1
      px = pos_x + vk * cos(yaw_k) * delta_t + poistion_noise1;
      // py = xk + vk*sin(yaw_angle)*delta_t + poistion_noise2
      py = pos_y + vk * sin(yaw_k) * delta_t + poistion_noise2;
      
    } else {
      // cout << "estimating px, py using integration" << endl;
      // px = xk + vk/yaw_rate_k *(sin(yaw_angle+yaw_rate*delta_t)-sin(yaw_angle)) + poistion_noise1
      px = pos_x + vk / yaw_rate_k *(sin(yaw_k + yaw_rate_k * delta_t ) - sin(yaw_k)) + poistion_noise1;
      // py = xk + vk/yaw_rate_k *(-cos(yaw_angle+yaw_rate*delta_t)+cos(yaw_angle)) + poistion_noise2
      py = pos_y + vk / yaw_rate_k *(-cos(yaw_k + yaw_rate_k * delta_t ) + cos(yaw_k)) + poistion_noise2;
      
    }
    
    // v = vk + 0 + velocity_noise
    v = vk + 0.0 + velocity_noise;
    // yaw = yaw_k + yaw_rate * delta_t + yaw_noise
    

    yaw = yaw_k + yaw_rate_k * delta_t + yaw_noise;
    // yaw_rate = yaw_rate_k + 0 + yaw_noise
    yaw_rate = yaw_rate_k + 0 + yaw_rate_noise;     

    Xsig_pred_.col(i)(0)=px;
    Xsig_pred_.col(i)(1)=py;
    Xsig_pred_.col(i)(2)=v;
    Xsig_pred_.col(i)(3)=yaw;
    Xsig_pred_.col(i)(4)=yaw_rate;
    
  }  
  
  // cout << "Xsig_pred_ : " << endl << Xsig_pred_ << endl;
  
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.    
  
  */
  
  // cout << "in Prediction start" << endl;
  // create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  // generate signma points using augmented covariance matrix
  generateAugmentedSigmaPoints(& Xsig_aug);
  
  // Predict new state using signma points and non linear equaltions
  predictNewStateUsingSigmaPoints(Xsig_aug,delta_t);
  
  // cout << "in Prediction before predicting mean state" << endl;
  // cout << "x_ : " << x_ << endl; 
  //predicted state mean
  x_.fill(0.0);
  for (int i = 0; i < ( 2 * n_aug_ + 1 ); i++) {  //iterate over sigma points
    x_ = x_+ weights_(i) * Xsig_pred_.col(i);
  }
  // cout << "x_ : " << x_ << endl;  
  
  // cout << "in Prediction before predicting covariance P" << endl;
  //predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < ( 2 * n_aug_ + 1 ); i++) {  //iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
   
    // if (x_diff(3) > 10.0 || x_diff(3) < -10.0 ){
      // cout << i << " in Prediction before angle normalization " << endl;
      // cout << x_diff(3) << endl;    
      // cout << "Xsig pred :" << Xsig_pred_.col(i) << endl;
      // cout << "X_ :" << x_ << endl;
    // }
    
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    //cout << i <<" in Prediction after angle normalization " << endl;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose(); 
    
  }
  
  // cout << "X_" << endl << "X_" << endl;
  // cout << "P_" << endl << "P_" << endl;
  
  // cout << "in Prediction end" << endl;  
}


void UKF::generateSTMatrices(MatrixXd& S,MatrixXd& Tc, MatrixXd & Zsig,VectorXd & z_pred, int angle_index){
  
  //calculate innovation covariance matrix S  
  S.fill(0.0);
  for(int i=0;i<(2*n_aug_+1);i++) {
      
      VectorXd z_diff = Zsig.col(i) - z_pred;
      
      //angle normalization
      if(angle_index > -1 ){
        while(z_diff(angle_index)>M_PI) z_diff(angle_index)-=2.*M_PI;
        while(z_diff(angle_index)<-M_PI) z_diff(angle_index)+=2.*M_PI;
      }
      
      S = S + weights_(i) * z_diff * z_diff.transpose();      
      
  }
  
  // compute Tc
  Tc.fill(0.0);
  for(int i=0;i<(2*n_aug_+1);i++) {
      
    VectorXd x_diff = Xsig_pred_.col(i)-x_;
    
    while(x_diff(3)>M_PI) x_diff(3)-=2.*M_PI;
    while(x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
    
    VectorXd z_diff = Zsig.col(i)-z_pred;
    
    if(angle_index > -1 ){
      while(z_diff(angle_index)>M_PI) z_diff(angle_index)-=2.*M_PI;
      while(z_diff(angle_index)<-M_PI) z_diff(angle_index)+=2.*M_PI;
    }
    Tc+=weights_(i)*x_diff*z_diff.transpose();
    
  }
  
    
}


/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  
  */
  // number of mesured parameters
  int n_z = 2;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  //transform sigma points into measurement space
  double THRESH = 0.00001;
  double px=0.0;
  double py=0.0;
  Zsig.fill(0.0);
  for(int i=0;i<(2*n_aug_+1);i++){
    
    px=Xsig_pred_(0,i);
    py=Xsig_pred_(1,i);
    
    // don't need any conversion
    Zsig(0,i)=px;
    Zsig(1,i)=py;      
  }
 
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);  

  z_pred.fill(0.0);
  //calculate mean predicted measurement
  for(int i=0;i<(2*n_aug_+1);i++){
      z_pred+=weights_(i)*Zsig.col(i);
  }
 
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z); // 5x3
   
  int angle_index = -1; // there is no angle ajustment for lidar
   
  //calculate Kalman gain K;
  generateSTMatrices(S,Tc,Zsig,z_pred,angle_index);
  
  // mesaurement covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);
  R.fill(0.0);  
  R(0,0)=std_laspx_*std_laspx_;
  R(1,1)=std_laspy_*std_laspy_;
  
  // add mesurement noise 
  S = S + R;
  
  MatrixXd K = Tc*S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z =meas_package.raw_measurements_;
  
  VectorXd z_diff = z-z_pred;
    
  while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  x_=x_+K*z_diff;
  P_-=K*S*K.transpose();
    
  // cout << "in UpdateLidar end" << endl;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */

void UKF::UpdateRadar(MeasurementPackage meas_package) { 
 /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
 
  // cout << "in UpdateRadar start" << endl;
  // number of mesured parameters
  int n_z = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  //transform sigma points into measurement space
  double THRESH = 0.00001;
  Zsig.fill(0);
  for(int i=0;i<(2*n_aug_+1);i++){
      
      
      double px=Xsig_pred_(0,i);
      double py=Xsig_pred_(1,i);
      double v=Xsig_pred_(2,i);
      double yaw=Xsig_pred_(3,i);
      double yaw_rate=Xsig_pred_(4,i);
      
      double rho=sqrt(px*px+py*py);
      //double phi = (py == 0.0 && px == 0.0) ? 0.0 : atan2(py, px);
      double phi = atan2(py, px);

      // Avoid dividing by zero
      //double rho_dot = (rho > THRESH) ? ( px *cos(yaw)*v + py * sin(yaw) * v ) / rho : 0.0;
      double rho_dot = ( px *cos(yaw)*v + py * sin(yaw) * v ) / rho;
      
      Zsig(0,i)=rho;
      Zsig(1,i)=phi;
      Zsig(2,i)=rho_dot;
      
  }
  
  // cout << "Zsig x_" << endl << Zsig <<endl; 
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  z_pred.fill(0.0);
  //calculate mean predicted measurement
  for(int i=0;i<(2*n_aug_+1);i++){
      z_pred+=weights_(i)*Zsig.col(i);
  }
 
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z); // 5x3
   
  int angle_index = 1; // phi angle at index 1
   
  //calculate Kalman gain K;
  generateSTMatrices(S,Tc,Zsig,z_pred,angle_index);
  
  // mesaurement covariance matrix
  MatrixXd R = MatrixXd(n_z,n_z);  
  R.fill(0.0);
  R(0,0)=std_radr_*std_radr_;
  R(1,1)=std_radphi_*std_radphi_;
  R(2,2)=std_radrd_*std_radrd_;
  
  // add mesurement noise 
  S = S + R;
  
  MatrixXd K = Tc*S.inverse();
  
  //update state mean and covariance matrix
  VectorXd z =meas_package.raw_measurements_;
  
  VectorXd z_diff = z-z_pred;
    
  while(z_diff(1)>M_PI) z_diff(1)-=2.*M_PI;
  while(z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
  
  x_=x_+K*z_diff;
  P_-=K*S*K.transpose();
    
  double nis=z_diff.transpose()*S.inverse()*z_diff;

  cout << "nis = " << nis << endl;

  // cout << "in UpdateRadar end" << endl;
} 

