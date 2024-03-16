// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  // put all the ids in constants
  private final CANSparkMax m_kicky = new CANSparkMax(ShooterConstants.kKicky, MotorType.kBrushless);
  private final CANSparkMax m_shootNSuckUno = new CANSparkMax(ShooterConstants.kShootNSuckUno, MotorType.kBrushless);
  private final CANSparkMax m_shootNSuckDos = new CANSparkMax(ShooterConstants.kShootNSuckDos, MotorType.kBrushless);
  private final DigitalInput m_kickyLimitSwitch = new DigitalInput(ShooterConstants.kKickyLimitSwitch);

  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopKicky() {
    m_kicky.set(0);
  }

  public void kicky(double speed) {
    m_kicky.set(speed);
  }

  public void stopShoot() {
    m_shootNSuckUno.set(0); // -0.10 suck 0.10 shut
    m_shootNSuckDos.set(0);
  }

  public void shootShort() {
    m_shootNSuckUno.set(0.4); // -0.10 suck 0.10 shut
    m_shootNSuckDos.set(0.4);
  }

  public void shootFarther() {
    m_shootNSuckUno.set(0.6); // -0.10 suck 0.10 shut
    m_shootNSuckDos.set(0.6);
  }

  public void shoot(double speed) {
    m_shootNSuckUno.set(speed); // -0.10 suck 0.10 shut
    m_shootNSuckDos.set(speed);
  }

  public void suck() {
    if (!hasNote()) {
      m_shootNSuckUno.set(-0.1); // -0.10 suck 0.10 shut
      m_shootNSuckDos.set(-0.1);
      m_kicky.set(-0.2);
    } else {
      stopKicky();
      stopShoot();
    }
  }
  public boolean hasNote() {
    return m_kickyLimitSwitch.get();
  }
}