// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.lamphere6566.lib.SendableCANSparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule implements Sendable {
  private final SendableCANSparkMax m_driveMotor;
  private final SendableCANSparkMax m_turningMotor;

  private final CANcoder m_turningEncoder;
  private final RelativeEncoder m_driveEncoder;

  private final ProfiledPIDController m_drivePIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleDriveController,
      ModuleConstants.kIModuleDriveController, ModuleConstants.kDModuleDriveController,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleSpeedMetersPerSecond,
          ModuleConstants.kMaxModuleAccelerationMetersPerSecondSquared));

  private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(0.25156, 2.6507, 0.53777);
  private final SimpleMotorFeedforward m_turnFeedForward = new SimpleMotorFeedforward(0.38, 0.3961);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(
      ModuleConstants.kPModuleTurningController,
      ModuleConstants.kIModuleTurningController,
      ModuleConstants.kDModuleTurningController,
      new TrapezoidProfile.Constraints(
          ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.m_drivePIDController
   *
   * @param driveMotorChannel      The channel of the drive motor.
   * @param turningMotorChannel    The channel of the turning motor.
   * @param turningEncoderChannel  The channel of the turning encoder.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      boolean turningEncoderReversed,
      String name) {
    m_driveMotor = new SendableCANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SendableCANSparkMax(turningMotorChannel, MotorType.kBrushless);
    m_turningMotor.setInverted(true);

    m_driveMotor.setIdleMode(IdleMode.kBrake);
    m_driveMotor.setSmartCurrentLimit(ModuleConstants.kMaxDriveCurrent);
    m_turningMotor.setIdleMode(IdleMode.kBrake);
    m_turningMotor.setSmartCurrentLimit(ModuleConstants.kMaxTurnCurrent);

    m_turningEncoder = new CANcoder(turningEncoderChannel);

    m_driveEncoder = m_driveMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
    m_driveEncoder.setPositionConversionFactor(Constants.DriveConstants.kDriveScale);
    m_driveEncoder.setVelocityConversionFactor(Constants.DriveConstants.kDriveScale / 60);

    m_drivePIDController.setIZone(Constants.ModuleConstants.kIZoneModuleDriveController);
    m_drivePIDController.setTolerance(Constants.ModuleConstants.kToleranceModuleDriveController);
    m_drivePIDController.setIntegratorRange(-Constants.ModuleConstants.kIntegratorMaxModuleDriveController,
        Constants.ModuleConstants.kIntegratorMaxModuleDriveController);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turningPIDController.setIZone(Constants.ModuleConstants.kIZoneModuleTurningController);
    m_turningPIDController.setTolerance(Constants.ModuleConstants.kPositionToleranceModuleTurningController,
        Constants.ModuleConstants.kVelocityToleranceModuleTurningController);
    m_turningPIDController.setIntegratorRange(-Constants.ModuleConstants.kIntegratorMaxModuleTurningController,
        Constants.ModuleConstants.kIntegratorMaxModuleTurningController);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(getTurnAngle()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(getTurnAngle()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getTurnAngle());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = m_turningPIDController.calculate(getTurnAngle(),
        state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(
        driveOutput + m_driveFeedForward.calculate(state.speedMetersPerSecond));
    m_turningMotor.setVoltage(turnOutput + m_turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  private double getTurnAngle() {
    return m_turningEncoder.getAbsolutePosition().getValue() * 2 * Math.PI;
  }

  public double getTurnVelocity() {
    return m_turningEncoder.getVelocity().getValue() * 2 * Math.PI;
  }

  public String getSubsystem() {
    return SendableRegistry.getSubsystem(this);
  }

  public void addChild(String name, Sendable child) {
    SendableRegistry.addLW(child, getSubsystem(), name);
  }

  public void disable() {
    m_driveMotor.disable();
    m_turningMotor.disable();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.setActuator(true);
    builder.setSafeState(this::disable);

    builder.addDoubleProperty("driveSpeed", () -> getState().speedMetersPerSecond, null);
    builder.addDoubleProperty("turnAngle", () -> getState().angle.getDegrees(), null);
    builder.addDoubleProperty("turnVelocity", this::getTurnVelocity, null);

    addChild("driveMotor", m_driveMotor);
    addChild("driveController", m_drivePIDController);
    addChild("turningEncoder", m_turningEncoder);
    addChild("turningMotor", m_turningMotor);
    addChild("turningController", m_turningPIDController);
  }
}
