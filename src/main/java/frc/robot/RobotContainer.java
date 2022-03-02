// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Driving.Driving;
import frc.robot.commands.Driving.DriveToDistance;
import frc.robot.commands.Driving.TurnToAngle;
import frc.robot.commands.Driving.autonDrivingRoutine;
import frc.robot.commands.Shooting.Extaking;
import frc.robot.commands.Shooting.Intaking;
import frc.robot.commands.Shooting.Shoot;
import frc.robot.commands.Shooting.ShootAnyDistance;
import frc.robot.commands.Shooting.ShootingFullRoutine;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.limeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_dt = new DriveTrain();
  private final XboxController m_xc = new XboxController(Constants.XboxPort);
  private final limeLight m_limelight = new limeLight();
  private final Shooter m_shooter = new Shooter();
  private final Index m_index = new Index();
  

  private final Driving m_driving = new Driving (m_dt, m_xc);
  private final autonDrivingRoutine m_auton = new autonDrivingRoutine(m_dt);
  private final Intaking m_intaking = new Intaking(m_index);
  private final Extaking m_extaking = new Extaking(m_index);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    SmartDashboard.putNumber("DistanceToTravel", 0);
    SmartDashboard.putNumber("Shooting Ticks", 0);
    configureButtonBindings();
    SmartDashboard.putData(new DriveToDistance(m_dt, () -> SmartDashboard.getNumber("DistanceToTravel", 0)));
   // SmartDashboard.putData(new TurnToAngle(m_dt, () -> m_limelight.getYaw())); // angle


    }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_dt.setDefaultCommand(m_driving);

    new JoystickButton(m_xc, Constants.ButtonA)
      .whenHeld(new TurnToAngle(m_dt, () -> -m_limelight.yeeYawww()));

    new JoystickButton(m_xc, Constants.ButtonB) 
      .whenPressed(new TurnToAngle(m_dt, () -> 3));

    new JoystickButton(m_xc, Constants.ButtonX)
      .whenHeld(new ShootAnyDistance(m_limelight, m_shooter, m_index, m_dt) );

    new JoystickButton(m_xc, Constants.ButtonY)// Y to shoot
      .whenHeld(new ShootingFullRoutine(m_dt, m_shooter, m_limelight, 5.25, m_index));

    new JoystickButton(m_xc, Constants.RightBumper)
      .whenHeld(new Intaking(m_index));

    new JoystickButton(m_xc, Constants.LeftBumper)
      .whenHeld(new Extaking(m_index));

        
    new JoystickButton(m_xc, Constants.startButton) // on/off light to heaven (limelight)
      .whenPressed(() -> m_shooter.setVelocityMode(10000))
      .whenReleased(()-> m_shooter.setPercentMode(0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_auton;
  }
}
