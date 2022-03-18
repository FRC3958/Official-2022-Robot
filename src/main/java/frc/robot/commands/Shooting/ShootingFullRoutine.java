// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Driving.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
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
  Index m_index; 

  public ShootingFullRoutine(DriveTrain d, Shooter s, limeLight l, Index i) {
    m_dt = d ; 
    m_shoot = s; 
    m_lime = l; 
    m_index = i; 
    

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new TurnToAngle(m_dt, () -> -m_lime.yeeYawww()),
        new Shoot(m_shoot, () -> Constants.shooterTicksFromDistance(m_lime.getDistanceToTarget()), true, m_index)
      ),
      new Shoot(m_shoot, () -> Constants.shooterTicksFromDistance(m_lime.getDistanceToTarget()), false, m_index)
    );
  }


}
