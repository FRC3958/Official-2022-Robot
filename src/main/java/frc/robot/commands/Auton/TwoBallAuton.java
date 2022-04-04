// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Driving.DriveToDistance;
import frc.robot.commands.Shooting.Intaking;
import frc.robot.commands.Shooting.KickBack;
import frc.robot.commands.Shooting.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuton extends SequentialCommandGroup {
  /** Creates a new TwoBallAuton. */
  Shooter m_s;
  DriveTrain dt;
  Index m_i;
  public TwoBallAuton(Shooter s, DriveTrain d, Index i) {
    m_s = s;
    dt = d;
    m_i = i;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveToDistance(dt, () -> 1.75),
        new Intaking(m_i)
      ),
      new KickBack(m_i),
  
      new Shoot(m_s, () -> 10495, false, m_i)
    );
  }
}
