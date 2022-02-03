// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  private DriveTrain m_dt; 
  private double DistanceToTravel; 
  private double StartingX = 0; 
  private double StartingY = 0; 
  public DriveToDistance(DriveTrain d, double dtt) {
    m_dt = d; 
    DistanceToTravel = dtt; 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StartingX = m_dt.getCurrentX();
    StartingY = m_dt.getCurrentY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distanceTravelled = Math.sqrt(Math.pow(m_dt.getCurrentX()-StartingX, 2) + Math.pow(m_dt.getCurrentY()-StartingY, 2));
    double percentError = (Math.abs(DistanceToTravel)-distanceTravelled)/DistanceToTravel;
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 
    SmartDashboard.putNumber("abs % error", absPercentError); 


    double motorOutput = 0.5; 

    if(absPercentError<=1 && absPercentError > 0.8) {
      motorOutput = 1*(1-absPercentError) + 0.3; 
    } else if (absPercentError<0.25) {
      motorOutput = absPercentError + 0.25; 
    }

    motorOutput *= isBackwards ? 1 : -1; 

    SmartDashboard.putNumber("motor output", motorOutput);

    m_dt.arcadeDrive(motorOutput, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_dt.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distanceTravelled = Math.sqrt(Math.pow(m_dt.getCurrentX()-StartingX, 2) + Math.pow(m_dt.getCurrentY()-StartingY, 2));
    SmartDashboard.putNumber("distancet", distanceTravelled); 
    return (Math.abs(DistanceToTravel) - distanceTravelled) < 0.005; 
  }
}
