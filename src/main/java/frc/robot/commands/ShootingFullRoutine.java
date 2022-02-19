// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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

  public ShootingFullRoutine(DriveTrain d, Shooter s, limeLight l, double fixedShootingDistance, Index i) { //TODO overload constructor for shooting from anywhere/fixed distance
    m_dt = d ; 
    m_shoot = s; 
    m_lime = l; 
    m_index = i; 
    
    DoubleSupplier distanceToTravel = () -> fixedShootingDistance == 0 ? 0 : -(m_lime.getDistanceToTarget() - fixedShootingDistance);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SmartDashboard.putNumber("currentd", m_lime.getDistanceToTarget());
    SmartDashboard.putNumber("distanttt", distanceToTravel.getAsDouble());
    addCommands(
      new TurnToAngle(m_dt, () -> -m_lime.yeeYawww()),
      new TurnToAngle(m_dt, () -> -m_lime.yeeYawww()),
      new KickBack(m_index),
      new DriveToDistance(m_dt, distanceToTravel),
      new ParallelDeadlineGroup(
        new DriveToDistance(m_dt, distanceToTravel), 
        new Shoot(m_shoot, ()  -> Constants.FixedShootingSpeed, true, m_index)
      ),
      new ParallelCommandGroup(
        new Shoot(m_shoot, () -> Constants.FixedShootingSpeed, false, m_index)
    ));
  }


}
