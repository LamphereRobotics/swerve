// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax m_climbyUno = new CANSparkMax(ClimberConstants.kClimbyUno, MotorType.kBrushless);
  private final CANSparkMax m_climbyDos = new CANSparkMax(ClimberConstants.kClimbyDos, MotorType.kBrushless);
  /** Creates a new Climbing. */
  public ClimberSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climber (double value) {
    m_climbyUno.set(value);
    m_climbyDos.set(value);
  }
}
