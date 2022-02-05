// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.Driving;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.autonDrivingRoutine;
import frc.robot.subsystems.DriveTrain;
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

  private final Driving m_driving = new Driving (m_dt, m_xc);
  private final autonDrivingRoutine m_auton = new autonDrivingRoutine(m_dt);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    SmartDashboard.putNumber("DistanceToTravel", 0);
    configureButtonBindings();
    SmartDashboard.putData(new DriveToDistance(m_dt, -2.5));
    SmartDashboard.putData(new TurnToAngle(m_dt, m_limelight.yeeYawww())); // angle


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
      .whenPressed(() -> m_dt.resetHeading());

    new JoystickButton(m_xc, Constants.ButtonB) 
      .whenPressed(() -> m_dt.resetEncoders());

    new JoystickButton(m_xc, Constants.ButtonX)
      .whenPressed(() -> m_dt.resetOdometry());
    
    new JoystickButton(m_xc, Constants.startButton)
    .whenPressed(() -> m_limelight.setLED(true))
    .whenReleased(()-> m_limelight.setLED(false));

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
