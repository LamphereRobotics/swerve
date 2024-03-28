// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class HoldShooterAndFire extends Command {
	private final double m_leftSpeed;
	private final double m_rightSpeed;
	private final double m_kickySpeed;
	protected final ShooterSubsystem m_shooter;

	public HoldShooterAndFire(double leftSpeed, double rightSpeed, double kickySpeed, ShooterSubsystem shooter) {
		addRequirements(shooter);
		m_leftSpeed = leftSpeed;
		m_rightSpeed = rightSpeed;
		m_shooter = shooter;
		m_kickySpeed = kickySpeed;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_shooter.shoot(m_leftSpeed, m_rightSpeed);
		m_shooter.kicky(m_kickySpeed);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
