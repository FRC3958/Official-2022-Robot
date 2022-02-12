// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.limeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingFullRoutine extends SequentialCommandGroup {
  /** Creates a new ShootingFullRoutine. */
  DriveTrain m_dt; 
  Shooter m_shoot;
  limeLight m_lime;

  public ShootingFullRoutine(DriveTrain d, Shooter s, limeLight l) { //TODO overload constructor for shooting from anywhere/fixed distance
    m_dt = d ; 
    m_shoot = s; 
    m_lime = l; 
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToAngle(m_dt, () -> -m_lime.yeeYawww()),
      new ParallelDeadlineGroup(
        new DriveToDistance(m_dt, () -> 0.5), 
        new Shoot(m_shoot, () -> Constants.FixedShootingSpeed, false)
      ),
      new Shoot(m_shoot, () -> Constants.FixedShootingSpeed, true)
    );
  }
}
