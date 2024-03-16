// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberModule extends SubsystemBase {
  private final CANSparkMax m_climbyMotor;
  private final DigitalInput m_climbyLimitSwitch;
  private final RelativeEncoder m_encoder;

  /** Creates a new ClimberModule. */
  public ClimberModule(int motorId, int limitSwitchId, boolean invertMotor) {
    m_climbyMotor = new CANSparkMax(motorId, MotorType.kBrushless);
    m_climbyMotor.setInverted(invertMotor);
    m_climbyLimitSwitch = new DigitalInput(limitSwitchId);
    m_encoder = m_climbyMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ascend() {
    if (m_encoder.getPosition() < 100) {
      m_climbyMotor.set(0.06);

    } else {
      m_climbyMotor.set(0);
    }
  }

  public void descend() {
    if (!m_climbyLimitSwitch.get()) {
      m_climbyMotor.set(-0.06);
    } else {
      m_climbyMotor.set(0);
      m_encoder.setPosition(0);
    }
  }

  public void stop() {
    m_climbyMotor.set(0);
  }
}
