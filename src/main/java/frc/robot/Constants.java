// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

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
    public static final int kFrontLeftDriveMotorPort = 2;
    public static final int kRearLeftDriveMotorPort = 8;
    public static final int kFrontRightDriveMotorPort = 4;
    public static final int kRearRightDriveMotorPort = 6;

    public static final int kFrontLeftTurningMotorPort = 1;
    public static final int kRearLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 3;
    public static final int kRearRightTurningMotorPort = 5;

    public static final int kFrontLeftTurningEncoderPorts = 1;
    public static final int kRearLeftTurningEncoderPorts = 4;
    public static final int kFrontRightTurningEncoderPorts = 2;
    public static final int kRearRightTurningEncoderPorts = 3;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = false;
    public static final boolean kFrontRightTurningEncoderReversed = false;
    public static final boolean kRearRightTurningEncoderReversed = false;

    public static final int kGyroPort = 1;

    public static final double kDriveScale = (Units.inchesToMeters(4) * Math.PI) / 6.75;

    // If you call DriveSubsystem.drive() with a different period make sure to
    // update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = 0.5715;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.5715;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for
    // your robot.

    public static final double kMaxSpeedMetersPerSecond = 4.2; // 4.2
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 28.5;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 386.9717;
    public static final double kMaxModuleSpeedMetersPerSecond = 4.2;
    public static final double kMaxModuleAccelerationMetersPerSecondSquared = 4.2;

    public static final double kPModuleTurningController = 2.0;
    public static final double kIModuleTurningController = 5.0;
    public static final double kDModuleTurningController = 0.0;
    public static final double kPositionToleranceModuleTurningController = 0.1;
    public static final double kVelocityToleranceModuleTurningController = 0.5;
    public static final double kIZoneModuleTurningController = 2.0;
    public static final double kIntegratorMaxModuleTurningController = 2.0;

    public static final double kPModuleDriveController = 0.0065166;
    public static final double kIModuleDriveController = 0.0;
    public static final double kDModuleDriveController = 0.0;
    public static final double kIZoneModuleDriveController = 0.3;
    public static final double kIntegratorMaxModuleDriveController = 1.0;
    public static final double kToleranceModuleDriveController = 0.005;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorStickPort = 1;

    public static final double kDeadband = 0.15;
  }

  public static final class ShooterConstants {
    public static final int kKicky = 9;
    public static final int kKickyLimitSwitch = 0;
    public static final int kShootNSuckUno = 12;
    public static final int kShootNSuckDos = 13;
  }

  public static final class AimyConstants {
    public static final int kAimyUno = 10;
    public static final int kAimyDos = 11;
    public static final int kAimyCancoder = 5;
  }

  public static final class ClimberConstants {
    public static final int kClimbyUno = 14;
    public static final int kClimbyDos = 15;
    public static final int kClimbyLimitSwitchUno = 1;
    public static final int kClimbyLimitSwitchDos = 2;
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
}
