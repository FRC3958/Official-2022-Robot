// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  public final WPI_TalonFX frontright = new WPI_TalonFX(Constants.FrontRight);
  public final WPI_TalonFX frontleft = new WPI_TalonFX(Constants.FrontLeft);
  public final WPI_TalonFX backright = new WPI_TalonFX(Constants.BackRight);
  public final WPI_TalonFX backleft = new WPI_TalonFX(Constants.BackLeft);

  public final DifferentialDrive DiffD = new DifferentialDrive(backleft, backright);

  /** Creates a new DriveTrain. */
  public DriveTrain() {

    frontright.configFactoryDefault();
    frontleft.configFactoryDefault();
    backleft.configFactoryDefault();
    backright.configFactoryDefault();

    frontleft.follow(backleft);
    frontright.follow(backright);

    frontright.setNeutralMode(NeutralMode.Coast);
    frontleft.setNeutralMode(NeutralMode.Coast);
    backright.setNeutralMode(NeutralMode.Coast);
    backleft.setNeutralMode(NeutralMode.Coast);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double forward, double turn){
    DiffD.arcadeDrive(-turn, forward);
  }

}
