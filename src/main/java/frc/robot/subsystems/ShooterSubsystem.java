// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

	// put all the ids in constants
	private final CANSparkMax m_kicky = new CANSparkMax(ShooterConstants.kKicky, MotorType.kBrushless);
	private final CANSparkMax m_shootNSuckUno = new CANSparkMax(ShooterConstants.kShootNSuckUno, MotorType.kBrushless);
	private final CANSparkMax m_shootNSuckDos = new CANSparkMax(ShooterConstants.kShootNSuckDos, MotorType.kBrushless);
	private final DigitalInput m_kickyLimitSwitch = new DigitalInput(ShooterConstants.kKickyLimitSwitch);

	public ShooterSubsystem() {
		m_shootNSuckUno.setInverted(false);
		m_shootNSuckDos.setInverted(true);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("shooter-kicky-voltage", m_kicky.getAppliedOutput() * m_kicky.getBusVoltage());
		SmartDashboard.putNumber("shooter-left-voltage",
				m_shootNSuckUno.getAppliedOutput() * m_shootNSuckUno.getBusVoltage());
		SmartDashboard.putNumber("shooter-right-voltage",
				m_shootNSuckDos.getAppliedOutput() * m_shootNSuckDos.getBusVoltage());
		SmartDashboard.putBoolean("shooter-has-note", hasNote());
	}

	public void stopKicky() {
		setKicky(0.0);
	}

	public void setKicky(double speed) {
		m_kicky.set(speed);
	}

	public void stopShoot() {
		setShoot(0.0);
	}

	public void setShoot(double speed) {
		m_shootNSuckUno.set(speed);
		m_shootNSuckDos.set(speed);
	}

	public void stop() {
		stopKicky();
		stopShoot();
	}

	public void suck() {
		setShoot(-0.1);
		setKicky(-0.2);
	}

	public Command shootCommand(double shootSpeed) {
		return run(() -> {
			setShoot(shootSpeed);
			setKicky(0.0);
		}).withTimeout(0.5).andThen(run(() -> {
			setShoot(shootSpeed);
			setKicky(0.2);
		}).withTimeout(1.0), this.stopCommand());
	}

	public Command suckCommand() {
		return run(() -> {
			setShoot(-0.15);
			setKicky(-0.2);
		}).until(this::hasNote).andThen(stopCommand());
	}

	public boolean hasNote() {
		return m_kickyLimitSwitch.get();
	}

	public Command stopCommand() {
		return runOnce(this::stop);
	}
}