// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.limeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAnyDistance extends SequentialCommandGroup {
  /** Creates a new ShootAnyDistance. */
  limeLight m_lime;
  Shooter m_shooter; 
  Index m_intake; 
  DriveTrain m_dt;
  public ShootAnyDistance(limeLight l, Shooter s, Index i, DriveTrain d) {
    m_lime = l; 
    m_shooter = s;
    m_intake = i;
    m_dt = d;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new TurnToAngle(m_dt, () -> -m_lime.yeeYawww()),
      new ParallelCommandGroup(
        new Shoot(m_shooter, () -> m_lime.GetShooterTicks(), false, m_intake))
      );
  }
}
