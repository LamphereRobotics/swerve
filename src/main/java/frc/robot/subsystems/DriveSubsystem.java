// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGyroPort);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    System.out.println("reset gyro");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private final SysIdRoutine driveRoutine = new SysIdRoutine(new Config(),
      new Mechanism(this::voltageDrive, this::logDrive, this));

  private void logDrive(SysIdRoutineLog log) {
    m_frontLeft.logDrive(log, "front-left-drive");
    m_frontRight.logDrive(log, "front-right-drive");
    m_rearLeft.logDrive(log, "rear-left-drive");
    m_rearRight.logDrive(log, "rear-right-drive");
  }

  public Command dynamicDrive(SysIdRoutine.Direction direction) {
    return driveRoutine.dynamic(direction);
  }

  public Command quasistaticDrive(SysIdRoutine.Direction direction) {
    return driveRoutine.quasistatic(direction);
  }

  private void voltageDrive(Measure<Voltage> volts) {
    m_frontLeft.voltageDrive(volts);
    m_frontRight.voltageDrive(volts);
    m_rearLeft.voltageDrive(volts);
    m_rearRight.voltageDrive(volts);
  }

  private final SysIdRoutine spinRoutine = new SysIdRoutine(new Config(),
      new Mechanism(this::voltageSpin, this::logSpin, this));

  private void logSpin(SysIdRoutineLog log) {
    m_frontLeft.logSpin(log, "front-left-spin");
    m_frontRight.logSpin(log, "front-right-spin");
    m_rearLeft.logSpin(log, "rear-left-spin");
    m_rearRight.logSpin(log, "rear-right-spin");
  }

  private void voltageSpin(Measure<Voltage> volts) {
    m_frontLeft.voltageSpin(volts);
    m_frontRight.voltageSpin(volts);
    m_rearLeft.voltageSpin(volts);
    m_rearRight.voltageSpin(volts);
  }

  public Command dynamicSpin(SysIdRoutine.Direction direction) {
    return spinRoutine.dynamic(direction);
  }

  public Command quasistaticSpin(SysIdRoutine.Direction direction) {
    return spinRoutine.quasistatic(direction);
  }
}
