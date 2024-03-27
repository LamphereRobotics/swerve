// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
	private final ClimberModule m_climbyUno = new ClimberModule(ClimberConstants.kClimbyUno,
			ClimberConstants.kClimbyLimitSwitchUno, true);
	private final ClimberModule m_climbyDos = new ClimberModule(ClimberConstants.kClimbyDos,
			ClimberConstants.kClimbyLimitSwitchDos, false);

	/** Creates a new ClimberSubsystem. */
	public ClimberSubsystem() {
	}

	@Override
	public void periodic() {
		m_climbyUno.logToDashboard("climber-left");
		m_climbyDos.logToDashboard("climber-right");
	}

	public Command ascendCommand() {
		return new RunCommand(() -> {
			m_climbyUno.ascend();
			m_climbyDos.ascend();
		}, this);
	}

	public Command descendCommand() {
		return new RunCommand(() -> {
			m_climbyUno.descend();
			m_climbyDos.descend();
		}, this);
	}

	public Command stopCommand() {
		return new RunCommand(() -> {
			m_climbyUno.stop();
			m_climbyDos.stop();
		}, this);
	}
}
