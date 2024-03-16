// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

class ArmConfig {
  // Motors
  public static final int kAimyUno = 10;
  public static final int kAimyDos = 11;
  public static final boolean kAimyUnoInverted = true; // TODO: check inversion
  public static final boolean kAimyDosInverted = false; // TODO: check inversion

  // Encoder
  public static final int kAimyCancoder = 5;
  public static final double kInputOffsetDegrees = -30.0;

  // PID
  public static final double kP = 0.0; // TODO: set this coefficient
  public static final double kI = 0.0; // TODO: set this coefficient
  public static final double kD = 0.0; // TODO: set this coefficient
  public static final double kPositionToleranceDegrees = 0.5; // TODO: determine a reasonable tolerance
  public static final double kVelocityToleranceDegreesPerSecond = 0.1; // TODO: determine a reasonable tolerance
  public static final double kMinimumInputDegrees = 0.0;
  public static final double kMaximumInputDegrees = 360.0;

  // Feedforward
  public static final double kSVolts = 0.0; // TODO: get this coefficient
  public static final double kGVolts = 2.07; // TODO: check this pseudovalue from re-calc against arm
  public static final double kVVoltSecondPerRad = 0.29; // TODO: check this pseudovalue from re-calc against arm
  public static final double kAVoltSecondSquaredPerRad = 0.05; // TODO: check this pseudovalue from re-calc against arm

  // Motion profile
  public static final double kMaxDegreesPerSecond = 90.0; // TODO: determine a reasonable max velocity
  public static final double kMaxDegreesPerSecondSquared = kMaxDegreesPerSecond * 2.0; // TODO: determine a reasonable
                                                                                       // max acceleration
}

// PID, motion profile, measurement, and goal/setpoint are in degrees.
// Error is in degrees for position and degrees per second for velocity.
// Feedforward requires coefficients and setpoints in radians. Conversion is
// done on the setpoint, but coefficients are in radians because re-calc does
// not have the decimal precision for setting them in degrees (the values are
// much smaller).
// Everything in the class *should* be correct and the only changes needed
// should be in the config. Unless IZone or integral range needs to be added.
public class AimBotSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax m_leftMotor = new CANSparkMax(ArmConfig.kAimyUno, MotorType.kBrushless);
  private final CANSparkMax m_rightMotor = new CANSparkMax(ArmConfig.kAimyDos, MotorType.kBrushless);
  private final CANcoder m_encoder = new CANcoder(ArmConfig.kAimyCancoder);
  private final ArmFeedforward m_feedForward = new ArmFeedforward(ArmConfig.kSVolts, ArmConfig.kGVolts,
      ArmConfig.kVVoltSecondPerRad,
      ArmConfig.kAVoltSecondSquaredPerRad);

  public AimBotSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            ArmConfig.kP,
            ArmConfig.kI,
            ArmConfig.kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(ArmConfig.kMaxDegreesPerSecond, ArmConfig.kMaxDegreesPerSecondSquared)),
        ArmConfig.kInputOffsetDegrees);

    m_controller.enableContinuousInput(ArmConfig.kMinimumInputDegrees, ArmConfig.kMaximumInputDegrees);
    m_controller.setTolerance(ArmConfig.kPositionToleranceDegrees, ArmConfig.kVelocityToleranceDegreesPerSecond);

    m_leftMotor.setInverted(ArmConfig.kAimyUnoInverted);
    m_rightMotor.setInverted(ArmConfig.kAimyDosInverted);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
    // double actualOutput = output
    //     + m_feedForward.calculate(Units.degreesToRadians(setpoint.position), Units.degreesToRadians(setpoint.velocity));
    // m_leftMotor.setVoltage(actualOutput);
    // m_rightMotor.setVoltage(actualOutput);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.rotationsToDegrees(m_encoder.getAbsolutePosition().getValueAsDouble()) + ArmConfig.kInputOffsetDegrees;
  }

  public void lookUp() {
    m_leftMotor.set(0.1);
    m_rightMotor.set(0.1);
  }

  public void lookDown() {
    m_leftMotor.set(-0.1);
    m_rightMotor.set(-0.1);
  }

  public void stop() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
  }
}
