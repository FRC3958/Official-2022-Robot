// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.channels.GatheringByteChannel;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Driving.Driving;
import frc.robot.commands.Driving.PID;
import frc.robot.commands.Auton.FiveBallAuton;
import frc.robot.commands.Auton.OneBallAuton;
import frc.robot.commands.Auton.TwoBallAuton;
import frc.robot.commands.Driving.DriveToDistance;
import frc.robot.commands.Driving.TurnToAngle;
import frc.robot.commands.Shooting.Extaking;
import frc.robot.commands.Shooting.Intaking;
import frc.robot.commands.Shooting.KickBack;
import frc.robot.commands.Shooting.Shoot;
import frc.robot.commands.Shooting.ShootAnyDistance;
import frc.robot.commands.Shooting.ShootingFullRoutine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TriggerButton;
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
  private final DriveTrain m_dt;
  private final XboxController m_xc;
  private final XboxController m_operaterController;
  private final limeLight m_limelight;
  private final Shooter m_shooter;
  private final Index m_index;
  private final Climber m_climber;
  private final Compressor m_compressor;

  

  private final Driving m_driving; 

  private final FiveBallAuton m_fiveBall;
  private final OneBallAuton m_oneBall;
  private final TwoBallAuton m_twoBall;

  private final Intaking m_intaking;
  private final Extaking m_extaking;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_xc = new XboxController(Constants.XboxPort);
    m_dt = new DriveTrain();
    m_operaterController = new XboxController(Constants.XboxPortOne);
    m_limelight = new limeLight();
    m_shooter = new Shooter();
    m_index = new Index();
    m_climber = new Climber(); 
    m_compressor = new Compressor(Constants.PCMID, PneumaticsModuleType.CTREPCM);
  
    //m_compressor = new Compressor(Constants.PCMID, PneumaticsModuleType.CTREPCM);
    //m_ss = new Solenoid(Constants.PCMID, PneumaticsModuleType.CTREPCM, Constants.ChannelID);
    
    m_driving = new Driving (m_dt, m_xc);
    
    m_fiveBall = new FiveBallAuton(m_dt, m_shooter, m_limelight, m_index);
    m_twoBall = new TwoBallAuton(m_shooter, m_dt, m_index);
    m_oneBall = new OneBallAuton(m_shooter, m_dt, m_index);

    m_intaking = new Intaking(m_index);
    m_extaking = new Extaking(m_index);

    // Configure the button bindings
    SmartDashboard.putNumber("Shooting Ticks", 0);
    m_compressor.enableDigital();
    
    
    configureButtonBindings();
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

    // intaking
    new JoystickButton(m_xc, Constants.RightBumper)
    .whenHeld(new Intaking(m_index));

    new JoystickButton(m_xc, Constants.backButton)
      .whenPressed(() -> m_index.dropTheGates());

    new JoystickButton(m_xc, Constants.startButton)
      .whenPressed(() -> m_index.raiseTheGates());
      
    new JoystickButton(m_xc, Constants.PressingJoystickLeft)
    .whenPressed(() -> m_compressor.enableDigital())
    .whenReleased(() -> m_compressor.disable());
    
    // extaking
    new JoystickButton(m_xc, Constants.LeftBumper)
      .whenHeld(new Extaking(m_index));
    
      
    new TriggerButton(m_xc, 2)
      .whenHeld(new TurnToAngle(m_dt, () -> -m_limelight.yeeYawww()));

    new JoystickButton(m_xc, Constants.ButtonB)
      .whenHeld(new Shoot(m_shooter, () -> 10292.5, false, m_index));

    new JoystickButton(m_xc, Constants.ButtonX)
      .whenHeld(new ShootingFullRoutine(m_dt, m_shooter, m_limelight, m_index));

    new JoystickButton(m_xc, Constants.ButtonY)
      .whenHeld(new Shoot(m_shooter, () -> Constants.shooterTicksFromDistance(m_limelight.getDistanceToTarget()), false, m_index));

    //new JoystickButton(m_xc, Constants.startButton)
    //  .whenHeld(new KickBack(m_index));

    new JoystickButton(m_operaterController, Constants.ButtonA)
      .whenPressed(() -> m_climber.pullUpDown(-0.3))
      .whenReleased(() -> m_climber.pullUpDown(0));

    new JoystickButton(m_operaterController, Constants.ButtonB) 
      .whenPressed(() -> m_climber.turnArm(-0.30))
      .whenReleased(() -> m_climber.turnArm(0));

      // shooting
    new JoystickButton(m_operaterController, Constants.ButtonX)
      .whenPressed(() -> m_climber.turnArm(0.325))
      .whenReleased(() -> m_climber.turnArm(0));

    new JoystickButton(m_operaterController, Constants.ButtonY)// Y to shoot
      .whenPressed(() -> m_climber.pullUpDown(0.3))
      .whenReleased(() -> m_climber.pullUpDown(0)); 
        
    new JoystickButton(m_operaterController, Constants.startButton) // on/off light to heaven (limelight)
      .whenPressed(() -> m_climber.resetEncoders());

    new JoystickButton(m_operaterController, Constants.backButton) 
      .whenPressed(() -> m_climber.setArmPosition(0))
      .whenReleased(() -> m_climber.turnArm(0));

    new JoystickButton(m_operaterController, Constants.LeftBumper)
      .whenPressed(() -> m_index.openGateway())
      .whenReleased(() -> m_index.closeGateway());

      new JoystickButton(m_operaterController, Constants.RightBumper)
      .whenPressed(() -> m_index.reverseGateway())
      .whenReleased(() -> m_index.closeGateway());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_fiveBall;
  }
}
