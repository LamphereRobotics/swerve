// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // put all the ids in constants
  private final CANSparkMax m_kicky = new CANSparkMax(ShooterConstants.kkicky, MotorType.kBrushless);
  private final CANSparkMax m_aimyUno = new CANSparkMax(ShooterConstants.kAimyUno, MotorType.kBrushless);
  private final CANSparkMax m_aimyDos = new CANSparkMax(ShooterConstants.kAimyDos, MotorType.kBrushless);
  private final CANSparkMax m_shootNSuckUno = new CANSparkMax(ShooterConstants.kShootNSuckUno, MotorType.kBrushless);
  private final CANSparkMax m_shootNSuckDos = new CANSparkMax(ShooterConstants.kShootNSuckDos, MotorType.kBrushless);

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void aimer(double value) {
    m_aimyDos.set(value);
    m_aimyUno.set(value);
  }

  public void shootnNSuck(double value) {
    m_shootNSuckUno.set(value);
    m_shootNSuckDos.set(value);
    m_kicky.set(value * 0.25);
  }
}
