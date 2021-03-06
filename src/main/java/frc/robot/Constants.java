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
    //DriveTrain constants
    public static final int FrontRight = 0;
    public static final int FrontLeft = 1;
    public static final int BackRight = 2;
    public static final int BackLeft = 3;
    public static final double DriveTrainkP = 0.1;
    public static final double DriveTrainkI = 0.000;
    public static final double DriveTrainkD = 0;
    public static final int kTimeout = 30;
    public static final int QuadEncoderResolution = 2048;
    public static final double WheelDiameter = 0.102;


    //controller constants
    public static final int XboxPort = 0;    
    public static final int ButtonA = 1;
	public static final int ButtonB = 2;
    public static final int ButtonX = 3;
    public static final int ButtonY = 4;
    public static final int startButton = 8;
    public static final int RightBumper = 6;
    public static final int LeftBumper = 5;
    public static final int backButton = 7;
    public static final int XboxPortOne = 1;


    //shooter constants
    public static final double ShooterkF = 0.05668365488; //0.05303465488
    public static final double ShooterkP = 0.27; //0.22
    public static final double ShooterkI = 0; //0
    public static final double ShooterkD = 0.5; //0.5
    public static final int ShooterTopID = 20;
    public static final int ShooterBottomID = 21;
    public static final int GatewayID = 11;
    public static final double AcceptableShootingError = 60;
    public static final double FixedShootingSpeed = 10800;


    //climber constants 
    public static final int ClimberLeftID = 23;
    public static final int ClimberRightID = 22;
    public static final int ClimberTurnID = 4;
    public static final int ClimberRightTurnID = 10;  
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
    public static final int climberStaticLeftLimitChannel = 4;
    public static final int climberStaticRightLimitChannel = 0;
    public static final int climberDynamicLeftLimitChannel = 3;
    public static final int climberDynamicRightLimitChannel = 1;
    public static final double ArmPassOffHeight = 0;


    public static double nativeUnitsToDegrees(double nu) {
        return ((nu/2048) * 360) * .111111111;
    }

    public static double degreesToNativeUnits(double d) {
        return ((d/360) * 2048) / .111111111; 
    }

    public static double shooterTicksFromDistance(double d) {
                                               //y intercept term  
                                               // ctrl+s then ctrl+shift+p to deploy (make sure you are connected in driver station)
        return ((d*d) * 62.7368) + (d * -76.2304) + 9900.62; //quadratic regression of shooting speeds from known distances (9050.62 y-intercept)
    }


    //limelight constants
    public static final double LimelightDegree = 13.55;
    public static final double HubHeight = 2.64;
    public static final double LimelightHeight = 0.84;


    //indexer constants
    public static final int IndexID = 16;
    public static final int PCMID = 0;
    public static final int RelayPortID = 0;
    public static final int PressingJoystickLeft = 9;

    
    
}
