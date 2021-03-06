// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index;

public class Intaking extends CommandBase {
  Index I;
  
  /** Creates a new Intaking. */
  public Intaking(Index m_I) {
    I = m_I;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    I.intake(1);
    I.dropTheGates();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    I.intake(0);
    I.raiseTheGates();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
