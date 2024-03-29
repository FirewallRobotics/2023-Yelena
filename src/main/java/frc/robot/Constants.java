// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.75);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 1;
    public static final int kRearRightTurningCanId = 2;

    public static final boolean kGyroReversed = false;

    // Balance specifications
    public static final double kGyroAngleRange = 5;
    public static final double kBalanceSpeedMultiplier = 0.08;
    public static final double kBalanceDecelerationDistance = 10;

    // Drive speeds
    public static final double kDefaultSpeed = 0.45;
    public static final double kSprintSpeed = 1.0;
    public static final double kSneakSpeed = 0.3;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the
    // bevel pinion
    public static final double kDrivingMotorReduction =
        (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps =
        (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor =
        ((kWheelDiameterMeters * Math.PI) / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor =
        (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput =
        kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    // public static final int kDriverJoystickPort = 0;
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.15;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = .8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double startingX1 = 13.4;
    public static final double startingX2 = 13.4;
    public static final double startingX3 = 13.4;
    public static final double startingX4 = 2.5;
    public static final double startingX5 = 2.5;
    public static final double startingX6 = 2.5;

    public static final double startingY1 = 3.0;
    public static final double startingY2 = 2.0;
    public static final double startingY3 = 4.0;
    public static final double startingY4 = 3.0;
    public static final double startingY5 = 2.0;
    public static final double startingY6 = 4.0;

    public static final double kAdjustSpeedMultiplier = 0.3;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Driving to the Balance Board
    public static final double kDriveToBalanceSpeedMultiplier = 0.2;
    public static final double kDriveToBalanceGyroAngleRange = 15;
    public static final int kDriveToBalanceDelay = 10;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmConstants {

    public static final int kMasterArmMotorPort = 9;
    public static final int kMinionArmMotorPort = 10;
    public static final double kMidHeight = .155;
    public static final double kMaxHeight = .077;
    public static final double kGrabbingHeight = .837;
    public static final double kDefaultHeight = 0;
    public static final double kArmUpSpeed = .5;
    public static final double kArmDownSpeed = -.4;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int kExtSolPort1 = 7;
    public static final int kExtSolPort2 = 4;
    public static final int kClawSolPort1 = 2;
    public static final int kClawSolPort2 = 3;
  }

  public static final class VisionConstants {

    public static final double kCameraCenterX = 285;
    public static final double kTargetCenterXRange = 10;
    public static final double kConeTargetWidth = 50;
    public static final double kConeTargetWidthRange = 10;
    public static final double kCubeTargetRadius = 50;
    public static final double kCubeTargetRadiusRange = 10;
    public static final double kAprilTagConeTargetX = 300;
    public static final double kAprilTagConeTargetXRange = 10;
    public static final double kAprilTagConeTargetArea = 100;
    public static final double kAprilTagConeTargetAreaRange = 10;
    public static final double kAprilTagCubeTargetX = 285;
    public static final double kAprilTagCubeTargetXRange = 10;
    public static final double kAprilTagCubeTargetArea = 100;
    public static final double kAprilTagCubeTargetAreaRange = 10;
    public static final double kDecelerationDistance = 9;

    public static final double kHighestXDifferenceLED = 30;
    public static final double kHighestZDifferenceLED = 30;
    public static final double kHighestTagAreaDifferenceLED = 30;
  }
}
