// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistance. */
  private DriveTrain m_dt; 
  private double DistanceToTravel = 0;
  private DoubleSupplier dttDS; // expression not static varible
  private double StartingX = 0; 
  private double StartingY = 0; 
  public DriveToDistance(DriveTrain d, DoubleSupplier dtt) {
    m_dt = d; 
    dttDS = dtt; 

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // starting values
    StartingX = m_dt.getCurrentX();
    StartingY = m_dt.getCurrentY();
    DistanceToTravel = dttDS.getAsDouble(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //find distance already traveled use a^2 + b^2 = c^2
    double distanceTravelled = Math.sqrt(Math.pow(m_dt.getCurrentX()-StartingX, 2) + Math.pow(m_dt.getCurrentY()-StartingY, 2));
    SmartDashboard.putNumber("distance travelled", distanceTravelled);
    SmartDashboard.putNumber("current x", m_dt.getCurrentX());
    SmartDashboard.putNumber("current y", m_dt.getCurrentY());
    // distence left to travel as %
    double percentError = (Math.abs(DistanceToTravel)-distanceTravelled)/DistanceToTravel;
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 
    


    double motorOutput = 0.6; 
    // y = mx+b
    if(absPercentError<=1 && absPercentError > 0.8) {// fine tuning 
      motorOutput = 1*(1-absPercentError) + 0.4; 
    } else if (absPercentError<0.25) {
      motorOutput = 1.4*absPercentError + 0.25; 
    }
    // if statement to find direction
    motorOutput *= isBackwards ? 1 : -1; 

    System.out.println("driving at " + motorOutput);

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
    return (Math.abs(DistanceToTravel) - distanceTravelled) < 0.005 || Math.abs(DistanceToTravel) < 0.1; //sets tolerence
  }
}
