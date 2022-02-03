// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TurnToAngle extends CommandBase {
  DriveTrain dt;
  private double startingAngle = 0;
  private double goalAngle;
  private double turningAngle;
  
  private double currentAngle;
  /** Creates a new TurnToAngle. */
  public TurnToAngle(DriveTrain drt, double gAngle) {
    dt = drt;
    goalAngle = gAngle;
    addRequirements(dt);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  startingAngle = dt.getHeading();
  
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = dt.getHeading();
    
    double percentError = -1* ((Math.abs(goalAngle) - (currentAngle- startingAngle))/goalAngle);
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 
    


    double motorOutput = 0.5; 

    if(absPercentError<=1 && absPercentError > 0.8) {
      motorOutput = 1*(1-absPercentError) + 0.3; 
    } else if (absPercentError<0.25) {
      motorOutput = absPercentError + 0.25; 
    }

    motorOutput *= isBackwards ? 1 : -1; 

    dt.arcadeDrive(0, motorOutput);


  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDrive(0,0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double percentError =  ((Math.abs(goalAngle) - (currentAngle- startingAngle))/goalAngle);
    SmartDashboard.putNumber("turn error", percentError); 
    return percentError > -.01 && percentError < .01; 
    
  }
}
