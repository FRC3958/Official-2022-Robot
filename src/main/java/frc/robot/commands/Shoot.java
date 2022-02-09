// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  /** Creates a new Shoot. */
  Shooter m_shooter;
  DoubleSupplier m_shootingSpeed; // expression that can give different values
  double shootingVelocity = 0; 
  boolean stopAtEnd = false; 
  
  public Shoot(Shooter s, DoubleSupplier shootingSpeed, boolean sae) {
    m_shooter = s;
    m_shootingSpeed = shootingSpeed;
    stopAtEnd = sae;
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
    m_shooter.setVelocityMode(shootingVelocity);
    // feeds the ball when the shooter is at an acceptable speed
    if(shootingVelocity - m_shooter.getShooterVelocity() < Constants.AcceptableShootingError) {
      m_shooter.openGateway();
    } else {
      m_shooter.closeGateway();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {// stops feeding balls
    if(stopAtEnd) {
      m_shooter.closeGateway();
    }
    m_shooter.setPercentMode(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
