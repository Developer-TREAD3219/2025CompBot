// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// TODO:Add constant doubles for each height. 
// They are placeholder right now so don't worry about setting a valude. Make sure they follow the pattern of the other constants.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

   //TODO RobotConfig Edit this when we get the robot configs 
    //public static RobotConfig config;

    
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24.5);  
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    //TODO: Set these values
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
//TODO: Verify pigeon ID
    public static final int kPigeonID = 15;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    //
    // TREAD is using a mid speed pinion gear in 2025.
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
    
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "April";
  }

//TODO: Set values for elevator constants
  public static final class ElevatorConstants {
    //TODO: ask the great bearded one what these ID's should be. I had to change them because the simulator yells at you if you try to use the same number twice
    public static final int KLeftElevatorID = 9;
    public static final int KRightElevatorID = 10;
    public static final int kElevatorBottomSwitchID = 0;  // DIO - when switched,   TODO: zero the elevator

    public static final double kDownPos = 0;
    public static final double kL1 = 0;
    public static final double kL2 = 0;
    public static final double kL3 = 0;
    public static final double kL4 = 0;
    public static final int kLimitSwitchPort = 0;
    public static final double kMaxVelocity = 0;
    
    public static final double kMaxAcceleration = 0;
    public static final double kP = 1.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kCountsPerInch = 0;
    public static final double kMaxPos = 0;
    public static final double kMax_output = 0;
    public static final double kBottomPos = 0;
    public static final double kPosTolerance = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kMinPos = 0;
  }
  //TODO: Changed intake servoID to avoid crashing in sim due to double assigned port issues kIntakeSeroID will need updating if we even use it
    public static final class coralDeliveryConstants {
    public static final int kIntakeServoID = 0;  // PWM port for the intake servo - MAY NOT BE USED
    public static final int kCoralDeliveryMotorID = 20;  // PWM port for the SparkMax/Neo that runs the intake
    public static final int kCoralInElevatorID = 1;  // DIO port 1 for the sensor that detects when a coral is in the elevator
    public static final int kCoralInPlaceID = 2;  // DIO port 2 for the sensor that detects when a coral is in place
    public static final double kIntakeServoOpen = 1.0;
    public static final double kIntakeServoClosed = 0.0;
   // TODO: need constant for intake and outtake speed 
    public static final double kIntakeSpeed = 0.50;
    public static final double kOuttakeSpeed = 0.50;
    
  }

  public static final class ClimberConstants {
    public static final int KClimberMotorID = 14;  // PWM port for the SparkMax/Neo that runs the climber
    public static final double kClimberSpeed = 0.5;
  }

}

