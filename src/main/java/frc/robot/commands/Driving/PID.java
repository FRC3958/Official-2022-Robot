// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PID extends CommandBase {
  /** Creates a new PID. */
  DriveTrain m_dt; 
  DoubleSupplier distant; 
  Double distancetogo = 0.0; 
  XboxController xc; 

  public PID(DriveTrain d, DoubleSupplier distance, XboxController x) {
    m_dt = d; 
    distant = distance;
    xc = x; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distancetogo = distant.getAsDouble();
    m_dt.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_dt.driveStraightToPos(-xc.getRightY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
