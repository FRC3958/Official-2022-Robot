// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public final WPI_TalonFX frontright = new WPI_TalonFX(Constants.FrontRight);
  public final WPI_TalonFX frontleft = new WPI_TalonFX(Constants.FrontLeft);
  public final WPI_TalonFX backright = new WPI_TalonFX(Constants.BackRight);
  public final WPI_TalonFX backleft = new WPI_TalonFX(Constants.BackLeft);

  public final DifferentialDrive DiffD = new DifferentialDrive(backleft, backright);

  private final AHRS m_ahrs = new AHRS(Port.kMXP); 
  private final DifferentialDriveOdometry odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    frontright.configFactoryDefault();
    frontleft.configFactoryDefault();
    backleft.configFactoryDefault();
    backright.configFactoryDefault();

    TalonFXConfiguration config1 = new TalonFXConfiguration();
    config1.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

      config1.slot0.kP = Constants.DriveTrainkP;
      config1.slot0.kI = Constants.DriveTrainkI;
      config1.slot1.kD = Constants.DriveTrainkD;

    TalonFXConfiguration config2 = new TalonFXConfiguration();
    config2.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

      config2.slot0.kP = Constants.DriveTrainkP;
      config2.slot0.kI = Constants.DriveTrainkI;
      config2.slot1.kD = Constants.DriveTrainkD;

    backleft.configAllSettings(config1);
    backright.configAllSettings(config2);

    frontleft.follow(backleft);
    frontright.follow(backright);

    frontright.setNeutralMode(NeutralMode.Coast);
    frontleft.setNeutralMode(NeutralMode.Coast);
    backright.setNeutralMode(NeutralMode.Coast);
    backleft.setNeutralMode(NeutralMode.Coast);

    resetEncoders();
    resetHeading();

    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftDistanceMeters(),
      getRightDistanceMeters());

    SmartDashboard.putNumber("heading", getHeading());
    SmartDashboard.putNumber("Distance X", getCurrentX());
    SmartDashboard.putNumber("Distance Y", getCurrentY());
  }

  public void arcadeDrive(double forward, double turn){
    DiffD.arcadeDrive(-turn, forward,true);
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getCurrentY() {
    return odometry.getPoseMeters().getY();
  }
  
  public double getCurrentX() {
    return odometry.getPoseMeters().getX();
  }

  public void resetEncoders() {
    backleft.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeout);
    backright.getSensorCollection().setIntegratedSensorPosition(0, Constants.kTimeout);
  }

  public double getMetersFromNative(double QuadEncoderInput) {
    return QuadEncoderInput*(Constants.WheelDiameter*Math.PI)/Constants.QuadEncoderResolution;
  }

  public double getLeftDistanceMeters() {
    return getMetersFromNative(backleft.getSelectedSensorPosition());
  }

  public double getRightDistanceMeters() {
    return getMetersFromNative(backright.getSelectedSensorPosition());
  }

  /**
   * Gets the angle of the bot
   * @return
   */
  public double getHeading() {
    return -m_ahrs.getAngle();
  }

  public void resetHeading() {
    m_ahrs.zeroYaw();
  }
  
}
