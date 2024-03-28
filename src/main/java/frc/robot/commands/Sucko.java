// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;

public class Sucko extends HoldShooterAndFire {
	/** Creates a new Sucko. */
	public Sucko(double leftSpeed, double rightSpeed, double kickySpeed, ShooterSubsystem shooter) {
		super(leftSpeed, rightSpeed, kickySpeed, shooter);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_shooter.hasNote();
	}
}
