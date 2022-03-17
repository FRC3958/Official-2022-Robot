// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{
	private final WPI_TalonFX climberLeft = new WPI_TalonFX(Constants.ClimberLeftID);
	private final WPI_TalonFX climberRight = new WPI_TalonFX(Constants.ClimberRightID);
	private final WPI_TalonSRX climberTurn = new WPI_TalonSRX(Constants.ClimberTurnID);
	private final TalonFXConfiguration  config = new TalonFXConfiguration(); 
	private final TalonSRXConfiguration turnconfig = new TalonSRXConfiguration(); 

	private final DigitalInput climberStaticLeft = new DigitalInput(Constants.climberStaticLeftLimitChannel);
	private final DigitalInput climberStaticRight = new DigitalInput(Constants.climberStaticRightLimitChannel);
	private final DigitalInput climberDynamicLeft = new DigitalInput(Constants.climberDynamicLeftLimitChannel);
	private final DigitalInput climberDynamicRight = new DigitalInput(Constants.climberDynamicRightLimitChannel);

	//private final AHRS navx = new AHRS(Port.kMXP); //TODO is reusing navx a good thing?
	
	public Climber() {
		climberLeft.configFactoryDefault();
		climberRight.configFactoryDefault();
		climberTurn.configFactoryDefault();

		config.slot0.kP = 0.5;
		config.slot0.kI = 0.005;
		config.slot0.kD = 5;

		turnconfig.slot0.kP = 0.5;
		turnconfig.slot0.kI = 0.0;

		config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

		turnconfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

		climberLeft.configAllSettings(config);
		climberRight.configAllSettings(config);
		climberTurn.configAllSettings(turnconfig);

		climberLeft.setNeutralMode(NeutralMode.Brake);
		climberRight.setNeutralMode(NeutralMode.Brake);
		climberTurn.setNeutralMode(NeutralMode.Brake);

		climberRight.setInverted(true);
		climberTurn.setInverted(true);

		climberRight.follow(climberLeft);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("arm position", getArmAngle());
		SmartDashboard.putNumber("arm height", getArmHeight());
		SmartDashboard.putBoolean("staticl arm on", staticArmOn());
		SmartDashboard.putBoolean("dynamicl arm on", dynamicArmOn());
		SmartDashboard.putBoolean("staticr arm on", !climberStaticRight.get());
		SmartDashboard.putBoolean("dynamicr arm on", !climberDynamicRight.get());
		///SmartDashboard.putNumber("robot pitch", getRobotPitch());
	}

	public void turnArm(double speed) {
		climberTurn.set(speed);
	}

	public void setArmPosition(double degrees) {
		climberTurn.set(ControlMode.Position, Constants.degreesToNativeUnits(degrees));
	}

	public void pullUpDown(double speed) {
		climberLeft.set(speed);
	}

	public void setArmHeight(double height) {
		climberLeft.set(ControlMode.Position, height);
	}

	public double getArmHeight() {
		return climberLeft.getSelectedSensorPosition();
	}

	public double getArmAngle() {
		return Constants.nativeUnitsToDegrees(climberTurn.getSelectedSensorPosition());
	}

	
	public boolean staticArmOn() {
		return !climberStaticLeft.get();
	}

	public boolean dynamicArmOn() {
		return !climberDynamicLeft.get();
	} 

	//public double getRobotPitch() {
	//	return navx.getPitch();
	//}

	public void stopAllMotors() {
		climberLeft.set(0);
		climberTurn.set(0);
	}

	public void resetEncoders() {
		climberLeft.getSensorCollection().setIntegratedSensorPosition(0, 30);
		climberRight.getSensorCollection().setIntegratedSensorPosition(0, 30);
		climberTurn.getSensorCollection().setQuadraturePosition(0, 30);
	}

	
}
