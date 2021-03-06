// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  // instantiating relevent motors
  private double motorSpeedDiff = 0;
  private WPI_TalonFX shooterTop = new WPI_TalonFX(Constants.ShooterTopID);
  private WPI_TalonFX shooterBottom = new WPI_TalonFX(Constants.ShooterBottomID);


  /** Creates a new Shooter. */
  public Shooter() {
    //sets motors to defult at startup
    shooterTop.configFactoryDefault();
    shooterBottom.configFactoryDefault();
    // gets encoder data for shooter motors
    shooterTop.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    shooterBottom.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.slot0.kF = Constants.ShooterkF;
    talonConfig.slot0.kP = Constants.ShooterkP;
    talonConfig.slot0.kI = Constants.ShooterkI;
    talonConfig.slot0.kD = Constants.ShooterkD;

    shooterTop.configAllSettings(talonConfig);
    shooterBottom.configAllSettings(talonConfig);

    shooterTop.setNeutralMode(NeutralMode.Coast);
    shooterBottom.setNeutralMode(NeutralMode.Coast);

    
    shooterTop.follow(shooterBottom); 
    shooterTop.setInverted(true);
  
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public double getShooterVelocity() {
    return shooterBottom.getSelectedSensorVelocity();// gets shooter speed
  }

  public void setPercentMode(double speed) {// insert % speed
    shooterBottom.set(ControlMode.PercentOutput, speed);
  }

  public void setVelocityMode(double speed) {// insert unit speed
    shooterBottom.set(ControlMode.Velocity, speed);
  }

  

  
}
