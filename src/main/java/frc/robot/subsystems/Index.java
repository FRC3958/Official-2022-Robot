// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
  public final WPI_TalonSRX index = new WPI_TalonSRX(Constants.IndexID);
  public final WPI_TalonSRX gateway = new WPI_TalonSRX(Constants.GatewayID);
  public final DoubleSolenoid intake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

  /** Creates a new Index. */
  public Index() {
    index.configFactoryDefault();
    gateway.configFactoryDefault();

    index.setNeutralMode(NeutralMode.Brake);
    gateway.setNeutralMode(NeutralMode.Brake);

    gateway.setInverted(InvertType.InvertMotorOutput);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake(double speed){
    index.set(ControlMode.PercentOutput, speed);
  }

  public void dropTheGates() {
    intake.set(Value.kReverse);
  }

  public void raiseTheGates() {
    intake.set(Value.kForward);
  }

  public void openGateway() {// feeds ball to shooter
    gateway.set(-0.5);
  }

  public void closeGateway() {// stops feeding ball to shooter
    gateway.set(0);
  }

  public void reverseGateway() {
    gateway.set(0.5);
  }

  public void reverseAll(){
    reverseGateway();
    intake(-0.3);
  }

  public void StopAll (){
    closeGateway();
    intake(0);
  }

  public void SendIt (){
    openGateway();
    intake(0.5);
  }

}
