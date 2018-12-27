package ascento_msgs;

public interface EstimatedStates extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ascento_msgs/EstimatedStates";
  static final java.lang.String _DEFINITION = "# estimated states message\n\n\n\n# Time of data acquisition\n\nHeader header\n\n\n\n# Data\n\nfloat32 est_mot_pos_left\n\nfloat32 est_mot_pos_right\n\nfloat32 est_mot_vel_left\n\nfloat32 est_mot_vel_right\n\nfloat32 est_hip_pos_left\n\nfloat32 est_hip_pos_right\n\nfloat32 est_hip_vel_left\n\nfloat32 est_hip_vel_right\n\nfloat32 est_hip_torque_left\n\nfloat32 est_hip_torque_right\n\nfloat32 est_imu_pos_roll\n\nfloat32 est_imu_pos_pitch\n\nfloat32 est_imu_pos_yaw\n\nfloat32 est_imu_vel_roll\n\nfloat32 est_imu_vel_pitch\n\nfloat32 est_imu_vel_yaw\n\nfloat32 est_imu_lin_acc_x\n\nfloat32 est_imu_lin_acc_y\n\nfloat32 est_imu_lin_acc_z\n\nfloat32 est_wheel_pos_left\n\nfloat32 est_wheel_pos_right\n\nfloat32 est_wheel_vel_left\n\nfloat32 est_wheel_vel_right\n\nfloat32 est_wheel_pos_left_prev\n\nfloat32 est_wheel_pos_right_prev\n\nfloat32 est_theta_left\n\nfloat32 est_theta_right\n\nfloat32 est_theta_mean\n\nfloat32 est_theta_dot_mean\n\nfloat32 est_filt_theta_dot_mean\n\nfloat32 est_lin_vel\n\nfloat32 est_filt_lin_vel\n\nfloat32 est_ang_vel\n\nfloat32 est_filt_ang_vel\n\nfloat32 est_robot_x_pos\n\nfloat32 est_robot_y_pos\n\nfloat32 est_robot_gamma_orient\n\nfloat32 est_robot_x_pos_prev\n\nfloat32 est_robot_y_pos_prev\n\nfloat32 est_robot_gamma_orient_prev\n\nfloat32 est_robot_rho\n\nfloat32 est_robot_psi\n\nfloat32 est_robot_beta\n\nfloat32 est_robot_s_pos\n\nfloat32 est_imu_drift_yaw\n\nfloat32 est_imu_pitch_offset\n\nfloat32 est_imu_pitch_offset_cad_error\n\nfloat32 est_filt_mot_pos_left\n\nfloat32 est_filt_mot_pos_right\n\nfloat32 est_filt_mot_pos_left_prev\n\nfloat32 est_filt_mot_pos_right_prev\n\nfloat32 est_mot_pos_ave\n\nfloat32 est_mot_vel_ave";
  static final boolean _IS_SERVICE = false;
  static final boolean _IS_ACTION = false;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  float getEstMotPosLeft();
  void setEstMotPosLeft(float value);
  float getEstMotPosRight();
  void setEstMotPosRight(float value);
  float getEstMotVelLeft();
  void setEstMotVelLeft(float value);
  float getEstMotVelRight();
  void setEstMotVelRight(float value);
  float getEstHipPosLeft();
  void setEstHipPosLeft(float value);
  float getEstHipPosRight();
  void setEstHipPosRight(float value);
  float getEstHipVelLeft();
  void setEstHipVelLeft(float value);
  float getEstHipVelRight();
  void setEstHipVelRight(float value);
  float getEstHipTorqueLeft();
  void setEstHipTorqueLeft(float value);
  float getEstHipTorqueRight();
  void setEstHipTorqueRight(float value);
  float getEstImuPosRoll();
  void setEstImuPosRoll(float value);
  float getEstImuPosPitch();
  void setEstImuPosPitch(float value);
  float getEstImuPosYaw();
  void setEstImuPosYaw(float value);
  float getEstImuVelRoll();
  void setEstImuVelRoll(float value);
  float getEstImuVelPitch();
  void setEstImuVelPitch(float value);
  float getEstImuVelYaw();
  void setEstImuVelYaw(float value);
  float getEstImuLinAccX();
  void setEstImuLinAccX(float value);
  float getEstImuLinAccY();
  void setEstImuLinAccY(float value);
  float getEstImuLinAccZ();
  void setEstImuLinAccZ(float value);
  float getEstWheelPosLeft();
  void setEstWheelPosLeft(float value);
  float getEstWheelPosRight();
  void setEstWheelPosRight(float value);
  float getEstWheelVelLeft();
  void setEstWheelVelLeft(float value);
  float getEstWheelVelRight();
  void setEstWheelVelRight(float value);
  float getEstWheelPosLeftPrev();
  void setEstWheelPosLeftPrev(float value);
  float getEstWheelPosRightPrev();
  void setEstWheelPosRightPrev(float value);
  float getEstThetaLeft();
  void setEstThetaLeft(float value);
  float getEstThetaRight();
  void setEstThetaRight(float value);
  float getEstThetaMean();
  void setEstThetaMean(float value);
  float getEstThetaDotMean();
  void setEstThetaDotMean(float value);
  float getEstFiltThetaDotMean();
  void setEstFiltThetaDotMean(float value);
  float getEstLinVel();
  void setEstLinVel(float value);
  float getEstFiltLinVel();
  void setEstFiltLinVel(float value);
  float getEstAngVel();
  void setEstAngVel(float value);
  float getEstFiltAngVel();
  void setEstFiltAngVel(float value);
  float getEstRobotXPos();
  void setEstRobotXPos(float value);
  float getEstRobotYPos();
  void setEstRobotYPos(float value);
  float getEstRobotGammaOrient();
  void setEstRobotGammaOrient(float value);
  float getEstRobotXPosPrev();
  void setEstRobotXPosPrev(float value);
  float getEstRobotYPosPrev();
  void setEstRobotYPosPrev(float value);
  float getEstRobotGammaOrientPrev();
  void setEstRobotGammaOrientPrev(float value);
  float getEstRobotRho();
  void setEstRobotRho(float value);
  float getEstRobotPsi();
  void setEstRobotPsi(float value);
  float getEstRobotBeta();
  void setEstRobotBeta(float value);
  float getEstRobotSPos();
  void setEstRobotSPos(float value);
  float getEstImuDriftYaw();
  void setEstImuDriftYaw(float value);
  float getEstImuPitchOffset();
  void setEstImuPitchOffset(float value);
  float getEstImuPitchOffsetCadError();
  void setEstImuPitchOffsetCadError(float value);
  float getEstFiltMotPosLeft();
  void setEstFiltMotPosLeft(float value);
  float getEstFiltMotPosRight();
  void setEstFiltMotPosRight(float value);
  float getEstFiltMotPosLeftPrev();
  void setEstFiltMotPosLeftPrev(float value);
  float getEstFiltMotPosRightPrev();
  void setEstFiltMotPosRightPrev(float value);
  float getEstMotPosAve();
  void setEstMotPosAve(float value);
  float getEstMotVelAve();
  void setEstMotVelAve(float value);
}
