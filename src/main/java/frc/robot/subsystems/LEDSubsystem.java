// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
	private final Spark m_ledController = new Spark(9);

	private double m_color = LEDConstants.kBlue;

	/** Creates a new LEDSubsystem. */
	public LEDSubsystem() {
	}

	@Override
	public void periodic() {
		m_ledController.set(m_color);
		SmartDashboard.putNumber("led-output", m_color);
	}

	public Command setColorCommand(double color) {
		return runOnce(() -> m_color = color);
	}

	public Command setRedCommand() {
		return setColorCommand(LEDConstants.kRed);
	}

	public Command setGreenCommand() {
		return setColorCommand(LEDConstants.kGreen);

	}

	public Command setBlueCommand() {
		return setColorCommand(LEDConstants.kBlue);

	}

	public Command setOrangeCommand() {
		return setColorCommand(LEDConstants.kOrange);

	}
}
