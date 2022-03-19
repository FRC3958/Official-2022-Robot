// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  Shooter m_shooter;
  DoubleSupplier m_shootingSpeed; // expression that can give different values
  double shootingVelocity = 0; 
  boolean reverseGateway = false; 
  Index I;
  
  public Shoot(Shooter s, DoubleSupplier shootingSpeed, boolean sae, Index In) {
    m_shooter = s;
    m_shootingSpeed = shootingSpeed;
    reverseGateway = sae;
    I = In;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootingVelocity = m_shootingSpeed.getAsDouble(); // gives usable double value
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shootingVelocity < 13000) {
    m_shooter.setVelocityMode(shootingVelocity);
    } else {
      m_shooter.setVelocityMode(12000);
    }
    // feeds the ball when the shooter is at an acceptable speed
    SmartDashboard.putNumber("Shooter Velocity", m_shooter.getShooterVelocity());
    if((Math.abs(shootingVelocity - m_shooter.getShooterVelocity()) < Constants.AcceptableShootingError) && !reverseGateway) {
      I.SendIt();
      
    } else if(reverseGateway) {
      I.reverseGateway();
      I.intake(0.3);
    } else {
      I.closeGateway();
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {// stops feeding balls
    I.StopAll();
    m_shooter.setPercentMode(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
