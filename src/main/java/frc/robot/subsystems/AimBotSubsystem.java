// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

class ArmConfig {
  // Positions
  public static final double kStoreArmDegrees = 0.0;

  // Motors
  public static final int kAimyUno = 10;
  public static final int kAimyDos = 11;
  public static final boolean kAimyUnoInverted = true;
  public static final boolean kAimyDosInverted = false;

  // Encoder
  public static final int kAimyCancoder = 5;
  public static final SensorDirectionValue kAimyCancoderDirection = SensorDirectionValue.Clockwise_Positive;
  public static final double kEncoderScaleFactor = 18.0 / 68.0;
  public static final double kInputOffsetDegrees = -29;

  // PID
  public static final double kP = 0.1;
  public static final double kI = 0.5;
  public static final double kD = 0.005;
  public static final double kPositionToleranceDegrees = 0.5;
  public static final double kVelocityToleranceDegreesPerSecond = 0.1;
  public static final double kIZone = 5.0;
  public static final double kIntegratorRange = 2.0;

  // Feedforward
  public static final double kSVolts = 0.16;
  public static final double kGVolts = 0.65;
  public static final double kVVoltSecondPerRad = 0.35;
  public static final double kAVoltSecondSquaredPerRad = 0.0;

  // Motion profile
  public static final double kMaxDegreesPerSecond = 360.0;
  public static final double kMaxDegreesPerSecondSquared = kMaxDegreesPerSecond * 4.0;
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
        ArmConfig.kStoreArmDegrees);

    m_controller.setTolerance(ArmConfig.kPositionToleranceDegrees, ArmConfig.kVelocityToleranceDegreesPerSecond);
    m_controller.setIZone(ArmConfig.kIZone);
    m_controller.setIntegratorRange(-ArmConfig.kIntegratorRange, ArmConfig.kIntegratorRange);

    m_leftMotor.setInverted(ArmConfig.kAimyUnoInverted);
    m_rightMotor.setInverted(ArmConfig.kAimyDosInverted);

    m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withSensorDirection(ArmConfig.kAimyCancoderDirection));

    enable();
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    if (m_enabled) {
      double actualOutput = output
          + m_feedForward.calculate(Units.degreesToRadians(getMeasurement()),
              Units.degreesToRadians(setpoint.velocity));
      m_leftMotor.setVoltage(actualOutput);
      m_rightMotor.setVoltage(actualOutput);
    } else {
      double actualOutput = m_feedForward.calculate(Units.degreesToRadians(getMeasurement()),
          0.0);
      m_leftMotor.setVoltage(actualOutput);
      m_rightMotor.setVoltage(actualOutput);
    }
  }

  @Override
  public double getMeasurement() {
    return Units.rotationsToDegrees(m_encoder.getPosition().getValueAsDouble() * ArmConfig.kEncoderScaleFactor)
        + ArmConfig.kInputOffsetDegrees;
  }

  public double getVelocity() {
    return Units.rotationsToDegrees(m_encoder.getVelocity().getValueAsDouble() * ArmConfig.kEncoderScaleFactor);
  }

  public double getOutputVoltage() {
    return m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
  }

  public Command setGoalCommand(double goal) {
    return new InstantCommand(() -> {
      setGoal(goal);
      enable();
    }, this);
  }

  public Command storeArmCommand() {
    return setGoalCommand(ArmConfig.kStoreArmDegrees);
  }
}
