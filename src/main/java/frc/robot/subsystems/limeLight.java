// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class limeLight extends SubsystemBase {
  // instantiates limelight (camera)
  public PhotonCamera camera = new PhotonCamera("3958Limelight");
  
// instantiates limelight and gets starting measurement
  PhotonPipelineResult result = camera.getLatestResult();
  private double yaw = 0; // measurement for left and right turns
  

  /** Creates a new limeLight. */
  public limeLight() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = new PhotonPipelineResult();// yaw constantly updates
    SmartDashboard.putNumber("yewYaw", yeeYawww()); // Outputs latest Yaw
    SmartDashboard.putNumber("distance", getDistanceToTarget());

     
  
  }

  public void setLED(boolean on){
      if(on){
        camera.setLED(VisionLEDMode.kOn);
      }else{
        camera.setLED(VisionLEDMode.kOff);
      }
  }

  public boolean lockOn(){
    return result.hasTargets();
  }

  public double yeeYawww(){
    result = camera.getLatestResult();// gets newest yaw
    if(result.hasTargets()) {//has it found the target
      return result.getBestTarget().getYaw();
    } else {
      return 0;

    }
  }

  public double getYaw() {
    return yaw;
  }

  public double getDistanceToTarget() {
    result = camera.getLatestResult();// gets newest yaw
    if(result.hasTargets()) {//has it found the target
      //TODO innacurrate
      return (Constants.HubHeight - Constants.LimelightHeight)/Math.tan(Math.toRadians(Constants.LimelightDegree + result.getBestTarget().getPitch()));
    } else {
      return 0;
    }
  }

}
