// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Aim;
import frc.robot.commands.Shoot;
import frc.robot.commands.Sucko;
import frc.robot.subsystems.AimBotSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

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
        private final AimBotSubsystem m_aimBot = new AimBotSubsystem();

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandJoystick m_operatorsStick = new CommandJoystick(OIConstants.kOperatorStickPort);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_robotDrive.zeroHeading();

                // Configure the button bindings
                configureButtonBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(m_robotDrive.driveTeleop(m_driverController));
                m_shooter.setDefaultCommand(m_shooter.stopCommand());
                m_climber.setDefaultCommand(m_climber.stopCommand());
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
                m_operatorsStick.button(1).whileTrue(new Aim(57, m_aimBot).andThen(new Shoot(.4, m_shooter)))
                                .onFalse(m_aimBot.storeArmCommand());
                m_operatorsStick.button(9).whileTrue(new Aim(80, m_aimBot).andThen(new Shoot(.09, m_shooter)))
                                .onFalse(m_aimBot.storeArmCommand());
                m_operatorsStick.button(5).whileTrue(new Aim(45, m_aimBot).andThen(new Shoot(.6, m_shooter)))
                                .onFalse(m_aimBot.storeArmCommand());
                m_operatorsStick.button(2).whileTrue(new Aim(30, m_aimBot).andThen(new Sucko(-0.15, -0.2, m_shooter)))
                                .onFalse(m_aimBot.storeArmCommand());
                m_operatorsStick.button(6).whileTrue(new Aim(70, m_aimBot))
                                .onFalse(m_aimBot.storeArmCommand());

                m_driverController.a().onTrue(m_robotDrive.resetGyro());
                m_driverController.leftTrigger().onTrue(m_robotDrive.setSlowModeCommand(true))
                                .onFalse(m_robotDrive.setSlowModeCommand(false));
                m_driverController.rightBumper().onTrue(m_robotDrive.setFieldRelativeCommand(true))
                                .onFalse(m_robotDrive.setFieldRelativeCommand(false));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(3, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(6, 1, new Rotation2d(0)),
                                config);

                var thetaController = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0),
                                thetaController,
                                m_robotDrive::setModuleStates,
                                m_robotDrive);

                // Reset odometry to the initial pose of the trajectory, run path following
                // command, then stop at the end.
                return Commands.sequence(
                                new InstantCommand(
                                                () -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
        }
}
