// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int FrontRight = 0;
    public static final int FrontLeft = 1;
    public static final int BackRight = 2;
    public static final int BackLeft = 3;
    public static final int XboxPort = 0;
    public static final double DriveTrainkP = 0.05;
    public static final double DriveTrainkI = 0.0025;
    public static final double DriveTrainkD = 0;
    public static final int kTimeout = 30;
    public static final int QuadEncoderResolution = 2048;
    public static final double WheelDiameter = 0.102;
    public static final int ButtonA = 1;
	public static final int ButtonB = 2;
    public static final int ButtonX = 3;
    public static final int ButtonY = 4;
    public static final int startButton = 8;
    public static final double ShooterkP = 0;
    public static final double ShooterkI = 0;
    public static final double ShooterkD = 0;
    public static final int ShooterTopID = 0;
    public static final int ShooterBottomID = 0;
    public static final int GatewayID = 0;
    public static final double AcceptableShootingError = 200;
    public static final double FixedShootingSpeed = 10000;
    public static final int ClimberLeftID = 0;
    public static final int ClimberRightID = 0;
    public static final int kTurnTravelUnitsPerRotation = 1;
    public static final int kEncoderUnitsPerRotation = 1;
    public static final double kGains_TurningkF = 0;
    public static final double kGains_TurningkP = 2;
    public static final double kGains_TurningkI = 0;
    public static final double kGains_TurningkD = 4;
    public static final double kGains_TurningkIzone = 200;
    public static final double kGains_TurningkPeakOutput = 1;
    public static final int kTimeoutMs = 30;
    public static final int kSlot_Turning = 1;
    public static final int PID_TURN = 1; 
}
