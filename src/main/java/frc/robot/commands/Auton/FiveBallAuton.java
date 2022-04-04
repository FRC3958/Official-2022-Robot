// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Driving.DriveToDistance;
import frc.robot.commands.Driving.TurnToAngle;
import frc.robot.commands.Driving.WaitFiveCommand;
import frc.robot.commands.Shooting.Intaking;
import frc.robot.commands.Shooting.KickBack;
import frc.robot.commands.Shooting.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.limeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallAuton extends SequentialCommandGroup {
  /** Creates a new autonDrivingRoutine. */
  DriveTrain dt;
  Shooter m_s;
  limeLight m_l;
  Index m_i; 
  public FiveBallAuton(DriveTrain dtt, Shooter s, limeLight l, Index i) {
    // Add your commands in the addCommands() call, e.g.
    dt = dtt;
    m_s = s;
    m_l = l;
    m_i = i; 
    // addCommands/(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveToDistance(dt, () -> 1.7),
        new Intaking(m_i)
      ),
      new TurnToAngle(dt, () -> 10),
      new KickBack(m_i),
  
      new InstantCommand(()->m_i.dropTheGates()),

      new ParallelDeadlineGroup(
        new WaitFiveCommand(),
        new Shoot(m_s, () -> 10595, false, m_i)
      ),

      new TurnToAngle(dt, ()->-30),
      new DriveToDistance(dt, ()-> 2.44)


    );
  }
}