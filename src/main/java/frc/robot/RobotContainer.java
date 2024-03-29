// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	private final ShooterSubsystem m_shooter = new ShooterSubsystem();
	private final ClimberSubsystem m_climber = new ClimberSubsystem();
	private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();

	// The driver's controller
	private final CommandXboxController m_driverController = new CommandXboxController(
			OIConstants.kDriverControllerPort);
	private final CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

	private final SendableChooser<Command> m_autonomousChooser = new SendableChooser<Command>();

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_robotDrive.zeroHeading();

		configureButtonBindings();
		configureAutonomousChooser();

		m_robotDrive.setDefaultCommand(m_robotDrive.driveTeleop(m_driverController));
		m_shooter.setDefaultCommand(m_shooter.stopCommand());
		m_climber.setDefaultCommand(m_climber.descendCommand());
		m_ledSubsystem.setDefaultCommand(Commands.either(m_ledSubsystem.setOrangeCommand(),
				m_ledSubsystem.setBlueCommand(), m_shooter::hasNote));
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
	 * subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
	 * passing it to a
	 * {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		m_operatorsStick.button(1).whileTrue(m_shooter.shootCommand(.4).alongWith(m_ledSubsystem.setRedCommand()));
		m_operatorsStick.button(2).whileTrue(m_shooter.suckCommand().alongWith(m_ledSubsystem.setGreenCommand()));

		m_operatorsStick.button(3).whileTrue(m_climber.ascendCommand());

		m_driverController.a().onTrue(m_robotDrive.resetGyro());
		m_driverController.leftTrigger().onTrue(m_robotDrive.setSlowModeCommand(true))
				.onFalse(m_robotDrive.setSlowModeCommand(false));
		m_driverController.rightBumper().onTrue(m_robotDrive.setFieldRelativeCommand(false))
				.onFalse(m_robotDrive.setFieldRelativeCommand(true));
	}

	private void configureAutonomousChooser() {
		m_autonomousChooser.setDefaultOption("drive forward",
				new AutoCommand(false, 0.0, 0.0, m_robotDrive, m_shooter));
		m_autonomousChooser.addOption("shoot and drive right",
				new AutoCommand(true, 120, 0.0, m_robotDrive, m_shooter));
		m_autonomousChooser.addOption("shoot and drive left",
				new AutoCommand(true, 240, 0.0, m_robotDrive, m_shooter));
		m_autonomousChooser.addOption("do nothing", Commands.none());
		SmartDashboard.putData(m_autonomousChooser);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return m_autonomousChooser.getSelected();
	}
}
