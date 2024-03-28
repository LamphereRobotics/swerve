// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
	private final Solenoid m_climbyUno = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.kClimbyUno);
	private final Solenoid m_climbyDos = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.kClimbyDos);

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void periodic() {
		SmartDashboard.putBoolean("climber-left", m_climbyUno.get());
		SmartDashboard.putBoolean("climber-right", m_climbyUno.get());
	}

	public Command ascendCommand() {
		return new RunCommand(() -> {
			m_climbyUno.set(true);
			m_climbyDos.set(true);
		}, this);
	}

	public Command descendCommand() {
		return new RunCommand(() -> {
			m_climbyUno.set(false);
			m_climbyDos.set(false);
		}, this);
	}
}
