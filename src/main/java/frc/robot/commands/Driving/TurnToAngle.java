// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.limeLight;

public class TurnToAngle extends CommandBase {
  DriveTrain dt;
  private double startingAngle = 0;
  public double goalAngle;
  private double turningAngle;
  
  private double currentAngle;
  private limeLight m_ll; 
  private boolean getAngleFromLimelight = false; 
  private boolean angleTooClose = false; 
  private DoubleSupplier limeYawDS;
  /** Creates a new TurnToAngle. */
  /*public TurnToAngle(DriveTrain drt, double gAngle) {
    dt = drt;
    goalAngle = gAngle;
    getAngleFromLimelight = false;
    addRequirements(dt);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }*/
  // turns robots depending on the limelight
  public TurnToAngle(DriveTrain drt, DoubleSupplier ds) {
    dt = drt; 
    getAngleFromLimelight = true; 
    limeYawDS = ds; 

    addRequirements(dt);
  }


/*  public TurnToAngle(DriveTrain drt, limeLight ll) {
    dt = drt;
    goalAngle = ll.yeeYawww();
    m_ll = ll;
    System.out.println(goalAngle + "Goal ANGLE");
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  startingAngle = dt.getHeading();
  if(getAngleFromLimelight) {
    //for some reason calling yeeYaw directly was restarting the roboRio, so I directly get the value from network tables (look at shuffleboard)
    goalAngle = limeYawDS.getAsDouble();//-1*NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("3958Limelight").getEntry("targetYaw").getDouble(0);
    if(Math.abs(goalAngle) < 6) { //6 degree tolerene
      angleTooClose = true;
      
    } else {
      angleTooClose = false; 
    }
    
  }
  
  }
 
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = dt.getHeading();
    
    double percentError =  -1*(goalAngle - (currentAngle- startingAngle))/Math.abs(goalAngle);
    boolean isBackwards = percentError<0; 
    double absPercentError = Math.abs(percentError); 
    

    
    double motorOutput = 0.45; 
    if(absPercentError<=1 && absPercentError > 0.8) {// tuned to turn 90 degrees
      motorOutput = 0.75*(1-absPercentError) + 0.3; 
    } else if (absPercentError<0.25) {
      motorOutput = 0.9*absPercentError + 0.225; 
    }

    double lowMotorOutput = 0.35;// tuned for anything under or = 45 degress
    if(goalAngle <= 45) {
      if(absPercentError <=1 && absPercentError > 0.8)
      lowMotorOutput = 0.5*(1-absPercentError) + 0.25; 
        else if (absPercentError < 0.6) {
          lowMotorOutput = 0.1667*absPercentError + 0.25; 
        }
      } 
      //if statments  to turn
    motorOutput *= isBackwards ? -1 : 1;
    lowMotorOutput *= isBackwards ? -1 : 1;

    // using tuned outputs to drive
    dt.arcadeDrive(0, motorOutput);
    dt.arcadeDrive(0, lowMotorOutput);


  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.arcadeDrive(0,0); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double percentError =  (goalAngle - (currentAngle- startingAngle))/Math.abs(goalAngle);
    // stop at this % range or when angleToClose is true
    return percentError > -.01 && percentError < .01 || angleTooClose; 
    
  }
}
