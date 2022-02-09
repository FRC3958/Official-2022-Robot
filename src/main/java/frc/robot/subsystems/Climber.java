// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  WPI_TalonFX climberLeft = new WPI_TalonFX(Constants.ClimberLeftID);
  WPI_TalonFX climberRight = new WPI_TalonFX(Constants.ClimberRightID);
  
  TalonFXConfiguration leftConfig = new TalonFXConfiguration(); 
  TalonFXConfiguration rightConfig = new TalonFXConfiguration();

  TalonFXInvertType leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"
  public Climber() {
    climberLeft.configFactoryDefault();
    climberRight.configFactoryDefault();

    climberLeft.setNeutralMode(NeutralMode.Brake);
    climberRight.setNeutralMode(NeutralMode.Brake);

    leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    rightConfig.remoteFilter1.remoteSensorDeviceID = climberLeft.getDeviceID(); //Device ID of Remote Source
		rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type

    setRobotTurnConfigs(rightInvert, rightConfig);

    leftConfig.peakOutputForward = +1.0;
		leftConfig.peakOutputReverse = -1.0;
		rightConfig.peakOutputForward = +1.0;
		rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for turn servo */
		/* FPID for Distance */
		rightConfig.slot1.kF = Constants.kGains_TurningkF;
		rightConfig.slot1.kP = Constants.kGains_TurningkP;
		rightConfig.slot1.kI = Constants.kGains_TurningkI;
		rightConfig.slot1.kD = Constants.kGains_TurningkD;
		rightConfig.slot1.integralZone = Constants.kGains_TurningkIzone;
		rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_TurningkPeakOutput;
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		
		/* APPLY the config settings */
		climberLeft.configAllSettings(leftConfig);
		climberRight.configAllSettings(rightConfig);
    climberRight.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		climberRight.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
		climberLeft.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);		//Used remotely by right Talon, speed up

    zeroEncoders();

    climberRight.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPercentOutput(double speed) {
    climberRight.set(ControlMode.PercentOutput, speed, DemandType.AuxPID, 0);
    climberLeft.follow(climberRight, FollowerType.AuxOutput1);
  }

  public void zeroEncoders() {
    climberLeft.getSensorCollection().setIntegratedSensorPosition(0, 30);
    climberRight.getSensorCollection().setIntegratedSensorPosition(0, 30);

  }

  void setRobotTurnConfigs(TalonFXInvertType masterInvertType, TalonFXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == TalonFXInvertType.Clockwise){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive heading?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity
				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct
				Will inverting the polarity give us a positive counterclockwise heading?
				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	}
}
